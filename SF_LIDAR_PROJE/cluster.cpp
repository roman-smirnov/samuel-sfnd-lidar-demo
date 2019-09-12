/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

//#include "render/render.h"
#include "render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree,float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);


void myprint (int i);
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render2DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;
		// split on x axis
		if(depth%3==0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		if(depth%3==1)
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}		
		if(depth%3==2)
		{
			//viewer->addLine(pcl::PointXYZ(window.x_min,window.y_min, node->point[2]),pcl::PointXYZ(window.x_max, window.y_max, node->point[2]),0,1,0,"line"+std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}


		iteration++;

		render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


	}

}

void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree,float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for(int id :  nearest){

		if(!processed[id])
			clusterHelper(id,points,cluster,processed,tree,distanceTol);

	}

}

//template<typename PointT>
//I want to use a genetic type here, but was getting compile errors.
std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance)
{	

  	auto startTime = std::chrono::steady_clock::now();
	std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

	
	std::vector<std::vector<float>> points;
	for(int i = 0; i < cloud->points.size(); i++)
	{	
		//each iteration make a new point, and assign xyz to it from cloud based on iteration int i
		std::vector<float> point;
		point.push_back(cloud->points[i].x);
		point.push_back(cloud->points[i].y);
		point.push_back(cloud->points[i].z);

		//push point to back of points
		points.push_back(point);
	}
	
	KdTree* tree = new KdTree;


    for (int i=0; i<points.size(); i++) {
    	tree->insert(points[i],i); 
	}
	
	    	std::vector<std::vector<int>> clusters_points = euclideanCluster(points, tree, clusterTolerance);

  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters_points)
  	{
            typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);
		  int count = 0;
  		for(int indice: cluster){
				pcl::PointXYZI point;
			  point.x = points[indice][0];
			  point.y = points[indice][1];
			  point.z = points[indice][2];
			  cloudCluster->points.push_back(point);
		  }
			

  			//clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		//renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		//++clusterId;
		  clusters.push_back(cloudCluster);
  	}

	  

/*
	      for(pcl::PointIndices getIndices: clusterIndices)
        {

            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
            for(int index:getIndices.indices)
                    cloudCluster->points.push_back(cloud->points[index]);

    cloudCluster->width = cloudCluster->points.size();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    clusters.push_back(cloudCluster);
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
        }
*/

	return clusters;
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	
	std::vector<bool> processed(points.size(),false);

	int i = 0;
	while( i< points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

	std::vector<int> cluster;
	clusterHelper(i, points, cluster, processed, tree, distanceTol);
	clusters.push_back(cluster);
	i++;
	}

	return clusters;
}


int main2 ()
{
	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min =   -10;
  	window.z_max =   10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7,5,.4}, {-6.3,8.4,-.5}, {-5.2,7.1,-1}, {-5.7,6.3,1.6}, {7.2,6.1,-2.4}, {8.0,5.3,-1.3}, {7.2,7.1,-1.3}, {0.2,-7.1,-1.3}, {1.7,-6.9,-1.3}, {-1.2,-7.2,-1.3}, {2.2,-8.9,-1.3} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(points);


	KdTree* tree = new KdTree;
  
    for (int i=0; i<points.size(); i++) {
    	tree->insert(points[i],i); 
	}


  	int it = 0;
  	render2DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search size" << std::endl;
  	std::vector<int> nearby = tree->search({-6,0,6.5,3},300.0);
	std::cout <<  nearby.size();


  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//


  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 5.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
