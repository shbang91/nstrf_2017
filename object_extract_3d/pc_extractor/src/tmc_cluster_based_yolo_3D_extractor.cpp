#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>

#include <tmc_yolo2_ros/Detections.h>
#include <tmc_yolo2_ros/Detection.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define CAMERA_PIXEL_WIDTH 640
#define CAMERA_PIXEL_HEIGHT 480
#define VOXEL_SIZE 0.01 //1cm voxels
#define CLUSTER_DIST_TOLERANCE 0.2
#define CLUSTER_BOXES_NAMESPACE "cluster_3D_boxes"
#define dobject_BOXES_NAMESPACE "dobject_3D_boxes"

#define KINECT_HEIGHT 1//0.7366 //m

#define CUTOFF_NUM 2 // Number of closest bounding boxes to consider

#define BOX_HEIGHT_TOL 0.1
#define BOX_WIDTH_TOL 0.1
#define BOX_DEPTH_TOL 0.1

#define BOX_HEIGHT_MAX_TOL 2.434
#define BOX_WIDTH_MAX_TOL 2.5
#define BOX_DEPTH_MAX_TOL 2.5

#define DEPTH_MAX 10.0

#define PURPLE_COLOR 45
#define GREEN_COLOR 0


#define	MAX_HEIGHT_TOL 2.434 //m Maximum height of person (8ft) to consider


class Cluster3D_BoundingBox{
public:
	float mean_x;
	float mean_y;
	float mean_z;

	float box_x_center; float box_y_center; float box_z_center;

	float x_min; float x_max;
	float y_min; float y_max;
	float z_min; float z_max;	

	pcl::PointIndices voxel_indices;
	int voxel_box_index;

	float box_x_size();
	float box_y_size();	
	float box_z_size();
	float distance_from_origin() const;

	Cluster3D_BoundingBox();
	Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices);
	~Cluster3D_BoundingBox();	

};
Cluster3D_BoundingBox::Cluster3D_BoundingBox(){}
Cluster3D_BoundingBox::~Cluster3D_BoundingBox(){}

Cluster3D_BoundingBox::Cluster3D_BoundingBox(const float _x_min, const float _x_max, 
						  const float _y_min, const float _y_max, 
						  const float _z_min, const float _z_max,
  						  const float _mean_x, const float _mean_y, const float _mean_z,
						  const int _voxel_box_index, const pcl::PointIndices &_voxel_indices):
			x_min(_x_min), x_max(_x_max),
			y_min(_y_min), y_max(_y_max),
			z_min(_z_min), z_max(_z_max),
			mean_x(_mean_x), mean_y(_mean_y), mean_z(_mean_z),
			voxel_box_index(_voxel_box_index){

	voxel_indices = _voxel_indices;

	box_x_center = (x_max + x_min)/2.0;
	box_y_center = (y_max + y_min)/2.0;
	box_z_center = (z_max + z_min)/2.0;

}

float Cluster3D_BoundingBox::box_x_size(){
	return std::abs(x_max - x_min);
}
float Cluster3D_BoundingBox::box_y_size(){
	return std::abs(y_max - y_min);
}
float Cluster3D_BoundingBox::box_z_size(){
	return std::abs(z_max - z_min);
}
float Cluster3D_BoundingBox::distance_from_origin() const{
	return sqrt(pow(box_x_center, 2) + pow(box_y_center, 2) + pow(box_z_center, 2));
}

struct Cluster3D_BoundingBox_distance_compare {
	bool operator() (const Cluster3D_BoundingBox &lhs, const Cluster3D_BoundingBox &rhs) const{ 
		float lhs_dist = lhs.distance_from_origin();
		float rhs_dist = rhs.distance_from_origin();		 
		return lhs_dist < rhs_dist;	
	}

} distance_compare_obj;


//std::sort (myvector.begin(), myvector.end(), distance_cost_compare_obj) 

class BoundingBox_Person_Desc
{
public:
  float tl_x;
  float tl_y;
  float width;
  float height;  

  BoundingBox_Person_Desc(float _tl_x, float _tl_y, 
  						  float _width, float _height); // Constructor
  ~BoundingBox_Person_Desc(); // Destructor
};
BoundingBox_Person_Desc::BoundingBox_Person_Desc(float _tl_x, float _tl_y, float _width, float _height):
							tl_x(_tl_x), 
							tl_y(_tl_y),  
							width(_width), 
							height(_height){}

BoundingBox_Person_Desc::~BoundingBox_Person_Desc(){}

class Bounding_Box_dobject{
public:
	ros::NodeHandle 		  node;

	ros::Publisher 			  prism_voxels_pub;
	ros::Publisher 			  cluster_voxels_pub;	

	ros::Publisher            cluster_boxes_array_pub; //bounding box for candidate dobjects
	ros::Publisher 			  dobject_boxes_array_pub; //bounding box for extracted dobject

	ros::Publisher 			  dobject_points_pub; // publish point cloud of dobject points;
	ros::Publisher 			  voxelized_pub; // publish point cloud of dobject points;	
	ros::Publisher 			  number_of_detected_dobjects_pub; //give number of detected dobjects



	ros::Subscriber			  detectedObjects_sub;
	ros::Subscriber			  yolo_detectedObjects_sub;	
	ros::Subscriber           registered_cloud_sub;

	std::vector<BoundingBox_Person_Desc> boxes;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::vector< pcl::PointCloud<pcl::PointXYZRGB> > bounding_box_points;
	std::vector< pcl::PointCloud<pcl::PointXYZ> > voxelized_bounding_box_points;	


	std::vector< std::vector<pcl::PointIndices> > bounding_boxes_clusters; // each element is a bounding box which contains a vector of cluster indices
	std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster

	std::vector<Cluster3D_BoundingBox> dobject_3D_boxes;


	visualization_msgs::MarkerArray cluster_boxes_array; 
	visualization_msgs::MarkerArray dobject_boxes_array;


    std::map<int, std::vector<int> > colormap;

	void yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg);	

  	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

  	void dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg);
  	void voxelize_points();
	void extract_clusters();

	void extract_candidate_dobject_boxes();

	void extract_dobject_boxes();

	void publish_dn_points();
	void publish_clusters();
	void publish_clusters_boxes();
	void publish_dobject_3D_boxes();
	void publish_dobject_points();	

	void delete_previous_markers();

	visualization_msgs::Marker createBoundingBoxMarker(const std::string &array_namespace,
													   const Cluster3D_BoundingBox &box, 
												  	   const int &marker_index, const int &apply_color);


	Bounding_Box_dobject(); // Constructor
	~Bounding_Box_dobject();  // Destructor

private:
  	bool init_vars();

};

Bounding_Box_dobject::Bounding_Box_dobject(): cloud(new pcl::PointCloud<pcl::PointXYZRGB>){
	init_vars();
}

bool Bounding_Box_dobject::init_vars(){

    std::string path = ros::package::getPath("villa_3d_object_extract");
    std::string filename = path + "/src/colors.txt";
    std::cout << filename << std::endl;
    FILE *fp = fopen(filename.c_str(),"rb"); 
    if(fp==NULL) {
        printf("Error trying to read file.\n");
    }

    size_t buffer_size = 80;
    char cmd[buffer_size]; // Character buffer
    int total_color_nums;

    // Get total number of color_nums
    int success = fscanf(fp, "%d", &total_color_nums); // This integer will be on the first line

    for (int i = 0; i < total_color_nums; i++){
      int color_num; int r; int g; int b;
      std::vector<int> color;
      int line_read_success = fscanf(fp, "%d %d %d %d", &color_num, &r, &g, &b);  
      color.push_back(r); color.push_back(g); color.push_back(b); 
      colormap[color_num] = color;

      printf("color_num: %d r: %d g: %d b: %d \n", color_num, r, g, b);

    }

    if (ferror(fp) != 0 || fclose(fp) != 0){
        return false;
    }

    printf(".....Successfully Loaded Colormap Parameters.\n");
    return true; 


}


Bounding_Box_dobject::~Bounding_Box_dobject(){}

void Bounding_Box_dobject::yolo_detected_obj_callback(const tmc_yolo2_ros::DetectionsConstPtr &msg){
	std::vector<tmc_yolo2_ros::Detection> objects;
	objects = msg->detections;	
	ROS_INFO("Hello world! I detected something");
	std::cout << "Number of detected objects: " << objects.size() << std::endl;

    int number_of_objects = objects.size();
    boxes.clear();
    for(size_t i = 0; i < objects.size(); i++){
        std::cout << "Object " << i+1 << std::endl;
        std::cout << "    Class Name:" << objects[i].class_name << std::endl;
        std::cout << "    Confidence:" << objects[i].confidence << std::endl;
        std::cout << "      Center x:" << objects[i].x << std::endl;  
        std::cout << "      Center y:" << objects[i].y << std::endl;
        std::cout << "        Height:" << objects[i].height << std::endl;
        std::cout << "         Width:" << objects[i].width << std::endl;                                          

        int left  = (objects[i].x - objects[i].width/2.0);
        int right = (objects[i].x + objects[i].width/2.0);
        int top   = (objects[i].y - objects[i].height/2.0);
        int bot   = (objects[i].y + objects[i].height/2.0);

        if (right > CAMERA_PIXEL_WIDTH-1)  right = CAMERA_PIXEL_WIDTH-1;
        if (bot > CAMERA_PIXEL_HEIGHT-1)    bot = CAMERA_PIXEL_HEIGHT-1;

        int tl_x = left < 0 ? 0 : left;
        int tl_y = top < 0 ? 0 : top;
        int width_bound = right - tl_x;
        int height_bound = bot - tl_y;

        std::cout << "    TL_x index:" << tl_x << std::endl;
        std::cout << "    TL_y index:" << tl_y << std::endl;        
        std::cout << "   width_bound:" << width_bound << std::endl; 
        std::cout << "  height_bound:" << width_bound << std::endl;                                                  

        // if(objects[i].class_name  == 'person')
        	boxes.push_back(BoundingBox_Person_Desc( tl_x, tl_y,  width_bound,  height_bound));
        //
    }
    
    std::cout << "Number of objects detected: " << boxes.size() <<std::endl;
    std_msgs::Int8 num_msg;
    if(dobject_boxes_array.markers.size() >= boxes.size() ){;
        num_msg.data = boxes.size();
        number_of_detected_dobjects_pub.publish(num_msg);
    }
            

/*    newObj.tl_x = left < 0 ? 0 : left;
    newObj.tl_y = top < 0 ? 0 : top;
    newObj.width = right - newObj.tl_x;
    newObj.height = bot - newObj.tl_y;*/


}

void Bounding_Box_dobject::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg){
	pcl::fromROSMsg(*msg, *cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_corners_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Set the same headers as original point cloud for simplicity   
	pt_corners_msg->header = (*cloud).header;

	// 
	// For each box, cluster points using euclidean clustering
	// for each box, extract dobject points using euclidean clustering
	//


	if (boxes.size() > 0){
		dn_extract_points(msg); 	// Extract points from each box.
		voxelize_points(); 			// filter points from each box.
		extract_clusters();			// Cluster the voxels
		extract_candidate_dobject_boxes(); // Create bounding boxes on all the clusters
		extract_dobject_boxes(); // Extract dobject boxes only;

		publish_dn_points();		// Visualize voxels bounded by dn box
		publish_clusters(); 		//Visualize Clusters
		publish_clusters_boxes();		
		publish_dobject_3D_boxes();
		publish_dobject_points();
	}


}

void Bounding_Box_dobject::dn_extract_points(const sensor_msgs::PointCloud2ConstPtr &msg){
	if (boxes.size() > 0){
		bounding_box_points.clear();

		// For each box, extract pointcloud bounded by the darknet classifier. store pointcloud to a vector
		for (int b_i = 0; b_i < boxes.size(); b_i++){		
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points (new pcl::PointCloud<pcl::PointXYZRGB>);
			prism_points->header = (*cloud).header;

			// Extract top left bounding box index
			int x = boxes[b_i].tl_x;
			int y = boxes[b_i].tl_y; 
			int tl_index = (y*CAMERA_PIXEL_WIDTH) + x;

			// Find points bounded by the dn box.
			int height = (int) boxes[b_i].height;		
			int width = (int) boxes[b_i].width;
			for (int m = 0; m < height; m++){
				for (int n = 0; n < width; n++){
					int point_index = tl_index + (m*CAMERA_PIXEL_WIDTH) + n;
				    // Prepare for copying
				    pcl::PointXYZRGB pt_color;

		    		if (!pcl::isFinite(cloud->points[point_index])){
		    			continue;
		    		}

			    	pt_color.x = cloud->points[point_index].x;
			      	pt_color.y = cloud->points[point_index].y;
			      	pt_color.z = cloud->points[point_index].z;
			        pt_color.rgb = cloud->points[point_index].rgb;

			        prism_points->points.push_back(pt_color);
				}
			}
			// Store vector of bounded point clouds 


			bounding_box_points.push_back(*prism_points);
		}//End of box for loop
	} 
}
void Bounding_Box_dobject::voxelize_points(){
	voxelized_bounding_box_points.clear();
	for(size_t b_i = 0; b_i < bounding_box_points.size(); b_i++){
		// Copy points to prepare for voxelization
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);		
		xyz_prism_points->header = (*cloud).header;

		for(size_t pt_i = 0; pt_i < bounding_box_points[b_i].points.size(); pt_i++){
		    pcl::PointXYZ pt_xyz;
	    	pt_xyz.x = bounding_box_points[b_i].points[pt_i].x;
	      	pt_xyz.y = bounding_box_points[b_i].points[pt_i].y;
	      	pt_xyz.z = bounding_box_points[b_i].points[pt_i].z;
	      	xyz_prism_points->points.push_back(pt_xyz);			
		}

		// Voxelize Points
	    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized_xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);			
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud( xyz_prism_points ); 
		// We set the size of every voxel to be 1x1x1cm (only one point per every cubic centimeter will survive).
		voxel_filter.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
		voxel_filter.filter(*voxelized_xyz_prism_points);

		// Check if voxelization succeeded
		//std::cout <<	 "Box ID" << b_i << " Original Points:" << bounding_box_points[b_i].points.size() << " "
		//		  << "Voxelized Points:" << voxelized_xyz_prism_points->size() << std::endl;

		// Store vector of voxelized bounded point cloud
		voxelized_bounding_box_points.push_back(*voxelized_xyz_prism_points);
	}	
}

void Bounding_Box_dobject::extract_clusters(){
	bounding_boxes_clusters.clear();
	for(size_t b_i = 0; b_i < voxelized_bounding_box_points.size(); b_i++){
		// Copy points to prepare for clustering
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_prism_points (new pcl::PointCloud<pcl::PointXYZ>);		
		xyz_prism_points->header = (*cloud).header;

		for(size_t pt_i = 0; pt_i < voxelized_bounding_box_points[b_i].points.size(); pt_i++){
		    pcl::PointXYZ pt_xyz;
	    	pt_xyz.x = voxelized_bounding_box_points[b_i].points[pt_i].x;
	      	pt_xyz.y = voxelized_bounding_box_points[b_i].points[pt_i].y;
	      	pt_xyz.z = voxelized_bounding_box_points[b_i].points[pt_i].z;
	      	xyz_prism_points->points.push_back(pt_xyz);			
		}

	    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   		tree->setInputCloud ( xyz_prism_points );

	    std::vector<pcl::PointIndices> cluster_indices; // Eg: cluster_indices[0] contains all the indices that belong to cluster 0

    	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    	ec.setClusterTolerance (CLUSTER_DIST_TOLERANCE); 
    	ec.setMinClusterSize (1);
    	ec.setMaxClusterSize (50000);
    	ec.setSearchMethod (tree);
    	ec.setInputCloud ( xyz_prism_points );
    	ec.extract (cluster_indices);
    	bounding_boxes_clusters.push_back(cluster_indices);
	    //std::cout << "Box id " << b_i << " has " << cluster_indices.size() << " clusters" << std::endl;
    }
//    std::cout << "Number of Bounding Boxes Clusters: " << bounding_boxes_clusters.size() << std::endl;
}


void Bounding_Box_dobject::extract_candidate_dobject_boxes(){
	bounding_box_clusters_3D.clear();
	// For each DN Box, extract the candidate dobjects
	for(size_t i = 0; i < bounding_boxes_clusters.size() ; i++){
		// For each cluster in the box
	    std::vector<Cluster3D_BoundingBox> vector_of_boxes_for_cluster; 

		for(size_t j = 0; j < bounding_boxes_clusters[i].size() ; j++){
			int box_index = i;
			int cluster_index = j;
			int n_clusters = bounding_boxes_clusters[box_index][cluster_index].indices.size();

  	      	float min_x = 10000;
		    float min_y = 10000;
		    float min_z = 10000;

		    float max_x = -10000;
		    float max_y = -10000;
		    float max_z = -10000;

		    int total_pts = 0;
		    float total_x = 0;
		    float total_y = 0;
		    float total_z = 0;		    

			// For each point in the cluster
			for(size_t k = 0; k < n_clusters; k++){
				int pt_index = bounding_boxes_clusters[box_index][cluster_index].indices[k];

		    	float pt_x = voxelized_bounding_box_points[box_index].points[pt_index].x;
		      	float pt_y = voxelized_bounding_box_points[box_index].points[pt_index].y;
		      	float pt_z = voxelized_bounding_box_points[box_index].points[pt_index].z;

      	        // Update minimum bounding box points
	        	if (pt_x < min_x){  min_x = pt_x; }
    	    	if (pt_y < min_y){  min_y = pt_y; }        
        		if (pt_z < min_z){  min_z = pt_z; }
        		// Update maximum bounding box points        
        		if (pt_x > max_x){  max_x = pt_x; }
        		if (pt_y > max_y){  max_y = pt_y; }        
        		if (pt_z > max_z){  max_z = pt_z; }

        		total_x += pt_x; total_y += pt_y; total_z += pt_z;
        		total_pts++;  
			} // Finished with iterating through the points

			if (total_pts > 0){
				float mean_x = total_x / ( (float) total_pts);
				float mean_y=  total_y / ( (float) total_pts);
				float mean_z = total_z / ( (float) total_pts);				
				vector_of_boxes_for_cluster.push_back(Cluster3D_BoundingBox(min_x, max_x, 
			  				min_y, max_y, min_z, max_z, mean_x, mean_y, mean_z,
							box_index, bounding_boxes_clusters[box_index][cluster_index]) );
			} // end if statement
		
		} // Finished with this cluster
		 bounding_box_clusters_3D.push_back(vector_of_boxes_for_cluster);
	} // Finished with this DN box
}

visualization_msgs::Marker Bounding_Box_dobject::createBoundingBoxMarker(const std::string &array_namespace,
																	   const Cluster3D_BoundingBox &box, 
																  	   const int &marker_index, const int &apply_color){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = (*cloud).header.frame_id; 
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = array_namespace;
    marker.id = marker_index;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position.x = box.box_x_center;//(box.x_max + box.x_min)/2.0;
    marker.pose.position.y = box.box_y_center;//(box.y_max + box.y_min)/2.0;
    marker.pose.position.z = box.box_z_center;//(box.z_max + box.z_min)/2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = std::abs(box.x_max - box.x_min);
    marker.scale.y = std::abs(box.y_max - box.y_min);
    marker.scale.z = std::abs(box.z_max - box.z_min);

    // Set the color -- be sure to set alpha to something non-zero!
    //int	apply_color = marker_index % colormap.size();

    marker.color.r = ((float) colormap[apply_color][0]) / 255.0;
    marker.color.g = ((float) colormap[apply_color][1]) / 255.0;
    marker.color.b = ((float) colormap[apply_color][2]) / 255.0;
    marker.color.a = 0.25;

    marker.lifetime = ros::Duration(1.0);

    return marker;
}


void Bounding_Box_dobject::extract_dobject_boxes(){
	dobject_3D_boxes.clear();
	// Returns the closest cluster found


	for(size_t i = 0; i < bounding_box_clusters_3D.size() ; i++){
		std::vector<Cluster3D_BoundingBox> current_dn_bounding_box;	

		if (bounding_box_clusters_3D[i].size() > 0){
			// Copy Bounding boxes
			for(size_t j = 0; j < bounding_box_clusters_3D[i].size() ; j++){
				current_dn_bounding_box.push_back(bounding_box_clusters_3D[i][j]);
			}
			// Sort Bounnding Boxes from Closest to Furthest
			std::sort (current_dn_bounding_box.begin(), current_dn_bounding_box.end(), distance_compare_obj);

			int cluster_box_index_indicating_dobject = 0; // The closest
			
			// Ignore if the "person" detected is further than 4m
			if (current_dn_bounding_box[cluster_box_index_indicating_dobject].mean_z > DEPTH_MAX){
				continue;
			}
	/*
			// Select only the top 3 closest boxes
			//current_dn_bounding_box.resize(CUTOFF_NUM);
				
			// Filter for minimum box size and maximum box height
			std::vector<int> indices_to_consider;
			for(size_t k = 0; k < current_dn_bounding_box.size() ; k++){
				float cb_size_x = current_dn_bounding_box[k].box_x_size();
				float cb_size_y = current_dn_bounding_box[k].box_y_size();			
				float cb_size_z = current_dn_bounding_box[k].box_z_size();
				float cb_max_height =  current_dn_bounding_box[k].y_max;

		       if ( ((cb_size_x >= BOX_WIDTH_TOL) && 
		       	     (cb_size_y >= BOX_HEIGHT_TOL) && 
		       	     (cb_size_z >= BOX_DEPTH_TOL)  && 
		       	     (cb_size_x < BOX_WIDTH_MAX_TOL) &&
		       	     (cb_size_y < BOX_HEIGHT_MAX_TOL) &&
	   	       	     (cb_size_z < BOX_DEPTH_MAX_TOL) &&    	       	     
		       	     (std::abs(cb_max_height + KINECT_HEIGHT) < MAX_HEIGHT_TOL) ) ){
		       			indices_to_consider.push_back(k);
		       }

			}

			
			// If no box satisfies this requirement, claim that the closest cluster must be dobject

			if (indices_to_consider.size() > 0){
				// There must be competing boxes. The tallest box at this point should be dobject
				// Now only select the box with the highest 
				float tallest_box_so_far = std::abs(current_dn_bounding_box[0].y_max); // highest so far
				int best_box_index = 0;
				for(size_t b_i = 0; b_i < indices_to_consider.size(); b_i++){

		  			 std::cout << "INDEX: " << b_i << " " << "dx dy dz" << 
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].x_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) << " " <<
					 std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].z_max) << " " << std::endl;


					if (  std::abs(current_dn_bounding_box[ indices_to_consider[b_i] ].y_max) > tallest_box_so_far){
						best_box_index = indices_to_consider[b_i];
						tallest_box_so_far = std::abs(current_dn_bounding_box[b_i].y_max);
					}

				}
				cluster_box_index_indicating_dobject = best_box_index;
			}
			
			*/
			
			dobject_3D_boxes.push_back(current_dn_bounding_box[cluster_box_index_indicating_dobject]);
		}

	}

	//std::cout << "Number of 3D Box dobjects: " << dobject_3D_boxes.size() << std::endl;

}




// void publish_dobject_bounding boxes


void Bounding_Box_dobject::publish_dn_points(){
	// Publish the voxelized points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr prism_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	prism_points_msg->header = (*cloud).header;
	int apply_color = 0;

	for(size_t vb_i = 0; vb_i < voxelized_bounding_box_points.size(); vb_i++){
		for(size_t pt_i = 0; pt_i < voxelized_bounding_box_points[vb_i].points.size(); pt_i++){
		    pcl::PointXYZRGB pt_color;
	    	pt_color.x = voxelized_bounding_box_points[vb_i].points[pt_i].x;
	      	pt_color.y = voxelized_bounding_box_points[vb_i].points[pt_i].y;
	      	pt_color.z = voxelized_bounding_box_points[vb_i].points[pt_i].z;

	        uint8_t r = (uint8_t) colormap[apply_color][0];
	        uint8_t g = (uint8_t) colormap[apply_color][1];
	        uint8_t b = (uint8_t) colormap[apply_color][2];    

	        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
			prism_points_msg->points.push_back(pt_color);
		}
		apply_color++;
		apply_color = apply_color % colormap.size();
	}

	prism_voxels_pub.publish(prism_points_msg);
}

void Bounding_Box_dobject::publish_clusters(){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	cluster_points_msg->header = (*cloud).header;

	// For each box
	for(size_t i = 0; i < bounding_boxes_clusters.size() ; i++){
		int apply_color = 0;
		// For each cluster in the box
//		std::cout << "number of boxes " <<  bounding_boxes_clusters.size() << std::endl;
		for(size_t j = 0; j < bounding_boxes_clusters[i].size() ; j++){
			int n_clusters = bounding_boxes_clusters[i][j].indices.size();
//			std::cout << "Cluster indices size " << n_clusters << std::endl;
			// For each point in the cluster
			for(size_t k = 0; k < n_clusters; k++){
				int box_index = i;
				int cluster_index = j;
				int pt_index = bounding_boxes_clusters[box_index][cluster_index].indices[k];

			    pcl::PointXYZRGB pt_color;
		    	pt_color.x = voxelized_bounding_box_points[box_index].points[pt_index].x;
		      	pt_color.y = voxelized_bounding_box_points[box_index].points[pt_index].y;
		      	pt_color.z = voxelized_bounding_box_points[box_index].points[pt_index].z;

				// Apply color to the cluster
		        uint8_t r = (uint8_t) colormap[apply_color][0];
		        uint8_t g = (uint8_t) colormap[apply_color][1];
		        uint8_t b = (uint8_t) colormap[apply_color][2];    

		        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
				cluster_points_msg->points.push_back(pt_color);
			}
			apply_color++; // Change color for the next cluster
			apply_color = apply_color % colormap.size();
		}
	}


	cluster_voxels_pub.publish(cluster_points_msg);


}

//
void Bounding_Box_dobject::publish_clusters_boxes(){
    cluster_boxes_array.markers.clear();
	delete_previous_markers();

	//std::vector< std::vector<Cluster3D_BoundingBox> > bounding_box_clusters_3D; // each element is a bounding box which contains a vector of 3d boxes for the cluster
    int marker_index = 0;
	for(size_t i = 0; i < bounding_box_clusters_3D.size() ; i++){
		int apply_color = marker_index;
		// For each cluster in the box
//		std::cout << "number of boxes " <<  bounding_boxes_clusters.size() << std::endl;
		for(size_t j = 0; j < bounding_box_clusters_3D[i].size() ; j++){
			cluster_boxes_array.markers.push_back(createBoundingBoxMarker(CLUSTER_BOXES_NAMESPACE, bounding_box_clusters_3D[i][j], marker_index, apply_color));
			marker_index++;
		}
	}
	cluster_boxes_array_pub.publish(cluster_boxes_array);

}

void Bounding_Box_dobject::publish_dobject_3D_boxes(){
    dobject_boxes_array.markers.clear();
	//delete_previous_markers();

    int marker_index = 0;
	int apply_color = 45;
	for(size_t i = 0; i < dobject_3D_boxes.size() ; i++){
		dobject_boxes_array.markers.push_back(createBoundingBoxMarker(dobject_BOXES_NAMESPACE, dobject_3D_boxes[i], marker_index, apply_color));
		marker_index++;
	}


	dobject_boxes_array_pub.publish(dobject_boxes_array);




}


void Bounding_Box_dobject::publish_dobject_points(){
	// Publish the voxelized points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dobject_points_msg (new pcl::PointCloud<pcl::PointXYZRGB>);
	dobject_points_msg->header = (*cloud).header;
	int apply_color = GREEN_COLOR;

	for(size_t i = 0; i < dobject_3D_boxes.size(); i++){
		int box_index = dobject_3D_boxes[i].voxel_box_index; 
		for(size_t j = 0; j < dobject_3D_boxes[i].voxel_indices.indices.size(); j++){
			int pt_index = dobject_3D_boxes[i].voxel_indices.indices[j];

		    pcl::PointXYZRGB pt_color;
	    	pt_color.x = voxelized_bounding_box_points[box_index].points[pt_index].x;
	      	pt_color.y = voxelized_bounding_box_points[box_index].points[pt_index].y;
	      	pt_color.z = voxelized_bounding_box_points[box_index].points[pt_index].z;

	        uint8_t r = (uint8_t) colormap[apply_color][0];
	        uint8_t g = (uint8_t) colormap[apply_color][1];
	        uint8_t b = (uint8_t) colormap[apply_color][2];    

	        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	        pt_color.rgb = *reinterpret_cast<float*>(&rgb);
			dobject_points_msg->points.push_back(pt_color);
		}

	}

	dobject_points_pub.publish(dobject_points_msg);
}


void Bounding_Box_dobject::delete_previous_markers(){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = (*cloud).header.frame_id; 
    marker.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = CLUSTER_BOXES_NAMESPACE;//"cluster_3D_boxes";
    marker.id = 0;
    marker.action = 3;//visualization_msgs::Marker::DELETEALL;
    cluster_boxes_array.markers.push_back(marker);
    cluster_boxes_array_pub.publish(cluster_boxes_array);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "villa_3D_object_bounding_boxes");

	Bounding_Box_dobject bbh_obj;

  	// Create Subsribers
	bbh_obj.yolo_detectedObjects_sub = bbh_obj.node.subscribe<tmc_yolo2_ros::Detections>("yolo2_node/detections", 10, boost::bind(&Bounding_Box_dobject::yolo_detected_obj_callback, &bbh_obj, _1));	
    bbh_obj.registered_cloud_sub = bbh_obj.node.subscribe<sensor_msgs::PointCloud2>("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 10, boost::bind(&Bounding_Box_dobject::cloud_callback, &bbh_obj, _1));

	bbh_obj.prism_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/prism_voxels", 0 );
	bbh_obj.cluster_voxels_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/cluster_voxels", 0 );
	bbh_obj.cluster_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/cluster_boxes", 0 );
	bbh_obj.dobject_boxes_array_pub = bbh_obj.node.advertise<visualization_msgs::MarkerArray>( "/dobject_boxes_3D", 0 );



	bbh_obj.dobject_points_pub = bbh_obj.node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >( "/dobject_voxels", 0 );

	bbh_obj.number_of_detected_dobjects_pub = bbh_obj.node.advertise<std_msgs::Int8>("detection/number_of_detected_dobjects", 0);



/*	  message_filters::Subscriber<geometry_msgs::TransformStamped> sub1_;
	  message_filters::Subscriber<geometry_msgs::TransformStamped> sub2_;
	  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
	  message_filters::Synchronizer<MySyncPolicy> sync_;

    sub1_(nh_, "topic1", 10),
    sub2_(nh_, "topic2", 10),
    sync_(MySyncPolicy(10),  sub1_, sub2_)
  
	sync_.registerCallback(boost::bind(&ExampleClass::Callback, this, _1, _2));	  
*/


	//Subscribe to detected objects
	//Subscribe to images
	//Subsribe to points




	ros::spin();
  return 0;
}
