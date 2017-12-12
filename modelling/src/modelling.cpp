#pragma omp parallel for
#include "modelling.h"


Modelling::Modelling(int id, string config, string name)
{
    if (name.find(".pcd")==-1)
        this->name = name + ".pcd";
    else
        this->name = name;
    this->id = id;
    this->properties_filename = config;
    printf("______________________________________________\n");
    printf("| ID:           %d\n", this->id);
    printf("| Object name:  %s\n", this->name.c_str());
    printf("| Config:       %s\n", this->properties_filename.c_str());
    printf("|____________________________________________|\n");
    initialize();
}
Modelling::~Modelling()
{
}
void Modelling::initialize()
{
//    this->sampling_properties = SamplingProperties(this->properties_filename, 1);
    this->poisson_properties = PoissonProperties(this->properties_filename);
    this->modelling_properties = ModellingProperties(this->properties_filename);
    this->clouds_filenames = this->modelling_properties.clouds_filename;
    this->matrix_filename = this->modelling_properties.matrix_filename;
    this->rotate_degree = this->modelling_properties.rotate_degree;
    this->clouds_num = this->clouds_filenames.size();

    this->pose_degree = this->modelling_properties.pose_degree;
    this->x_bias = this->modelling_properties.x_bias;
    this->x_degree = this->modelling_properties.x_degree;
    this->y_bias = this->modelling_properties.y_bias;
    this->y_degree = this->modelling_properties.y_degree;
    this->z_bias = this->modelling_properties.z_bias;
    this->z_degree = this->modelling_properties.z_degree;

    printf("______________________________________________\n");
    printf("|         MODELLING CONSTRUCTOR INFO         |\n");
    printf("| matrix filename:  %s\n", this->matrix_filename.c_str());
    printf("| pose degree:      %f\n", this->pose_degree);
    printf("| rotate degree:    %f\n", this->rotate_degree);
    printf("| file number:      %d\n", this->clouds_num);
    printf("|____________________________________________|\n");
    for (int i = 0; i < this->clouds_filenames.size(); i++)
    {
        this->clouds.push_back(loadPointCloud(this->clouds_filenames[i]));
    }
    this->matrix = readMatrix();
    printf("[INFO] Loading transform matrix\n");
    cout << this->matrix << endl;
    
}
void Modelling::rotatePointCloud(PointCloudT::Ptr &cloud, float degree, char mode)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    if (mode == 'z')
    {
        matrix(0,0) = cos(degree);
        matrix(0,1) = -sin(degree);
        matrix(1,0) = sin(degree);
        matrix(1,1) = cos(degree);
    }
    else if (mode == 'y')
    {
        matrix(0,0) = cos(degree);
        matrix(0,2) = -sin(degree);
        matrix(2,0) = sin(degree);
        matrix(2,2) = cos(degree);
    }
    else if (mode == 'x')
    {
        matrix(1,1) = cos(degree);
        matrix(1,2) = -sin(degree);
        matrix(2,1) = sin(degree);
        matrix(2,2) = cos(degree);
    }
    else
    {
        printf("[ERROR] Insert true mode index !\n");
    }
	pcl::transformPointCloud(*cloud, *cloud, matrix);
}
PointCloudT::Ptr Modelling::loadPointCloud ( string filename )
{
	PointCloudT::Ptr cloud( new PointCloudT );
	pcl::PCDReader reader;
	reader.read<PointT> ( filename, *cloud );
	return cloud;
}
Eigen::Matrix4f Modelling::readMatrix()
{    
    fstream in(this->matrix_filename.c_str());
    string line;
    Eigen::Matrix4f result;
    int index = 0;
    while(getline(in,line))
    {
        stringstream ss(line);
        float value;
        while(ss>>value)
        {
            result(index/4, index%4) = value;
            index++;
        }
    }
    return result;
}

void Modelling::preProcessPointCloud(PointCloudT::Ptr &cloud, Eigen::Matrix4f matrix, int index)
{
    Eigen::Matrix4f rotate = matrix;
    rotate(0, 3) += this->x_bias;
    rotate(1, 3) += this->y_bias;
    rotate(2, 3) += this->z_bias;
    pcl::transformPointCloud(*cloud, *cloud, rotate);

}
void Modelling::smoothAndSampling(PointCloudT::Ptr &cloud, 
    float search_radius, bool polynomial_fit, int polynomial_order, 
       float radius_search, int depth)
{
	pcl::MovingLeastSquares<PointT, PointT> mls;
	mls.setInputCloud (cloud);
	mls.setSearchRadius (search_radius);
	mls.setPolynomialFit (polynomial_fit);
	mls.setPolynomialOrder (polynomial_order);
	mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius (0.005);
	mls.setUpsamplingStepSize (0.003);
	PointCloudT::Ptr cloud_smoothed (new PointCloudT ());
	mls.process (*cloud_smoothed);
	pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_smoothed);
	ne.setRadiusSearch (radius_search);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud_smoothed, centroid);
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	ne.compute (*cloud_normals);
	for(size_t i = 0; i < cloud_normals->size (); ++i)
	{
	    cloud_normals->points[i].normal_x *= -1;
	    cloud_normals->points[i].normal_y *= -1;
	    cloud_normals->points[i].normal_z *= -1;
	}
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
	
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth (depth);
	poisson.setInputCloud (cloud_smoothed_normals);
	pcl::PolygonMesh mesh;
	poisson.reconstruct (mesh);
    // PointCloudT::Ptr result(new PointCloudT);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    // visualizePointCloud(result, RGB(255,0,0));
    // pcl::io::savePCDFileASCII("reconstruct_"+filename, *result);
}
void Modelling::removeOutlier(PointCloudT::Ptr &cloud, int meanK, int threshold)
{
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (meanK);
	sor.setStddevMulThresh (threshold);
	sor.setNegative(false);
	sor.filter (*cloud);
}

void Modelling::voxelGridFilter(PointCloudT::Ptr &cloud, float size)
{
	pcl::VoxelGrid<PointT> vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(size, size, size);
	vox.filter(*cloud);
}
PointCloudT::Ptr Modelling::getModel()
{
    PointCloudT temp;
    PointCloudT::Ptr model(new PointCloudT);
    vector<PointCloudT::Ptr> raw_clouds;
    float degree_threshold = this->rotate_degree * 1.25 / 180 * M_PI;
    for (int i = 0; i < this->clouds_num; i++)
    {
        rotatePointCloud(this->clouds[i], this->x_degree*M_PI/180, 'x');
        rotatePointCloud(this->clouds[i], this->y_degree*M_PI/180, 'y');
        rotatePointCloud(this->clouds[i], this->z_degree*M_PI/180, 'z');
        preProcessPointCloud(this->clouds[i], this->matrix, i+1);
    
        vector<PointCloudT::Ptr> compare;
        PointCloudT::Ptr temp1(new PointCloudT);
        *temp1 = * this->clouds[i];
        raw_clouds.push_back(temp1);
        compare.push_back(temp1);
        for (int j = 0; j < this->clouds[i]->points.size(); j++)
        {
            float d = atan(this->clouds[i]->points[j].x / this->clouds[i]->points[j].y);
            if ( d <= -degree_threshold || d >= degree_threshold || this->clouds[i]->points[j].y > 0.01)
            {
                this->clouds[i]->erase(this->clouds[i]->points.begin() + j);
                j--;
            }
        }
        //=======================debug===============================//
        for (int j = 0; j < this->clouds[i]->points.size(); j++)
		{
        	float rate = 6.0/7.0;
        	this->clouds[i]->points[j].x = rate * this->clouds[i]->points[j].x;
        	this->clouds[i]->points[j].y = rate * this->clouds[i]->points[j].y;
		}
        //===========================================================//
        compare.push_back(this->clouds[i]);
       // visualizePointCloud(compare);
        rotatePointCloud(this->clouds[i], this->rotate_degree*i*M_PI/180., 'z');
        rotatePointCloud(raw_clouds[i], this->rotate_degree*i*M_PI/180., 'z');
        stringstream ss;
        string index_str;
        ss << i;
        ss >> index_str;
        pcl::io::savePCDFile("part"+index_str+".pcd", *(this->clouds[i]));
        if (i == 0)
            temp = *this->clouds[i];
        temp += *this->clouds[i];
    }
    visualizePointCloud(this->clouds);
    *model = temp;

    pcl::io::savePCDFileASCII("origin.pcd", *model);

    vector<PointCloudT::Ptr> shows;
    PointCloudT::Ptr show_cloud1(new PointCloudT);
    PointCloudT::Ptr show_cloud2(new PointCloudT);
    PointCloudT::Ptr show_cloud3(new PointCloudT);
    *show_cloud1 = *model;
    shows.push_back(show_cloud1);
    visualizePointCloud(shows);

    OutlierProperties outlier_properties = OutlierProperties(this->properties_filename);
    removeOutlier(model, outlier_properties.meank, outlier_properties.threshold);

    *show_cloud2 = *model;
    shows.push_back(show_cloud2);
    visualizePointCloud(shows);

    VoxelGridProperties voxel_properties = VoxelGridProperties(this->properties_filename, 1);
    voxelGridFilter(model, voxel_properties.size);
    
    *show_cloud3 = *model;
    shows.push_back(show_cloud3);
    printf("[INFO] Raw points size: %d; removed points size: %d \n", (int)shows[0]->points.size(), (int)shows[shows.size()-1]->points.size());
    visualizePointCloud(shows);

    pcl::io::savePCDFileASCII(this->name, *model);
    
    // this->sampling_properties.info();
    // smoothAndSampling(model, this->sampling_properties.radius_search,
    //         this->sampling_properties.polynomial_fit, 
    //         this->sampling_properties.polynomial_order, 
    //         this->sampling_properties.radius_search, 
    //         this->sampling_properties.depth);
    
    // OutlierProperties outlier_properties = OutlierProperties(this->properties_filename);
    // removeOutlier(model, outlier_properties.meank, outlier_properties.threshold);
    // visualizePointCloud(model, RGB(127,0,127));

    // smooth the surface
    printf("[INFO] Starting Smoothing Process... \n");
    pcl::MovingLeastSquares<PointT, PointT> mls;
	mls.setInputCloud (model);
	mls.setSearchRadius (this->poisson_properties.search_radius);//0.015);
	mls.setPolynomialFit (this->poisson_properties.polynomial_fit);//true);
	mls.setPolynomialOrder (this->poisson_properties.polynomial_order);//5);
	mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius (0.005);
	mls.setUpsamplingStepSize (0.003);
	PointCloudT::Ptr cloud_smoothed (new PointCloudT);
	mls.process (*cloud_smoothed);
    *model = *cloud_smoothed;

    shows.push_back(model);
    visualizePointCloud(shows);

    printf("[INFO] Starting Sampling Process... \n");
    this->poisson_properties.info();
    poissonSampling(model, this->poisson_properties.ksearch,
            this->poisson_properties.confidence,
            this->poisson_properties.degree,
            this->poisson_properties.depth,
            this->poisson_properties.ISO,
            this->poisson_properties.manifold,
            this->poisson_properties.polygon,
            this->poisson_properties.smooth,
            this->poisson_properties.scale,
            this->poisson_properties.solver,
            this->poisson_properties.min_depth
    );

    // this->sampling_properties.info();
    // smoothAndSampling(model, this->sampling_properties.radius_search,
    //         this->sampling_properties.polynomial_fit, 
    //         this->sampling_properties.polynomial_order, 
    //         this->sampling_properties.radius_search, 
    //         this->sampling_properties.depth);
    
    // PointCloudT::Ptr cloud_smoothed(new PointCloudT);
    // pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
	// ne.setNumberOfThreads (8);
	// ne.setInputCloud (model);
	// ne.setRadiusSearch (this->sampling_properties.radius_search);
	// Eigen::Vector4f centroid;
	// compute3DCentroid (*cloud_smoothed, centroid);
	// ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
	// pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	// ne.compute (*cloud_normals);
	// for(size_t i = 0; i < cloud_normals->size (); ++i)
	// {
	//     cloud_normals->points[i].normal_x *= -1;
	//     cloud_normals->points[i].normal_y *= -1;
	//     cloud_normals->points[i].normal_z *= -1;
	// }
	
	// pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
	// pcl::concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
	
	// pcl::Poisson<pcl::PointNormal> poisson;
	// poisson.setDepth (this->sampling_properties.depth);
	// poisson.setInputCloud (cloud_smoothed_normals);
	// pcl::PolygonMesh mesh;
	// poisson.reconstruct (mesh);
    // // PointCloudT::Ptr result(new PointCloudT);
    // pcl::fromPCLPointCloud2(mesh.cloud, *model);


    printf("[INFO] Deleting bad points... \n");
    PointCloudT::Ptr new_model(new PointCloudT);
    for (int i = 0; i < model->points.size(); i++)
        if (model->points[i].z >= 0.0)
            new_model->push_back(model->points[i]);
		
    // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (new_model);
    // std::vector<pcl::PointIndices> cluster_indices;
	// pcl::EuclideanClusterExtraction<PointT> ec;
	// ec.setClusterTolerance (0.0015); // 2cm
	// ec.setMinClusterSize (2);
	// ec.setMaxClusterSize (50);
	// ec.setSearchMethod (tree);
	// ec.setInputCloud (new_model);
	// ec.extract (cluster_indices);
    // PointCloudT::Ptr show1 (new PointCloudT);
    // *show1 = *new_model;
	// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	// {
	// 	PointCloudT::Ptr cloud_cluster (new PointCloudT);
	// 	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	// 	{
    //         new_model->points.erase(new_model->points.begin()+*pit);
    //         new_model->width --;
    //     }
    //         // cloud_cluster->points.push_back (new_model->points[*pit]);
    //     // *new_model = *new_model - *cloud_cluster;
	// }
    pcl::io::savePCDFileASCII("reconstructed_"+this->name, *new_model);
    // PointCloudT::Ptr clouds[2] = {show1,new_model};
    // RGB rgbs1[2] = {RGB(255,0,0), RGB(255,255,255)};
    // visualizePointCloud(clouds,rgbs1,2);
    vector<PointCloudT::Ptr> show_new_model;
    show_new_model.push_back(new_model);
    visualizePointCloud(show_new_model);

    // extract slices
    printf("[INFO] Modelling Process Finished. \n");
    printf("[INFO] Starting KeyPoints Extraction Process ... \n");
    FeatureExtraction feature(this->id, this->name, this->properties_filename);
    printf("[INFO] KeyPoints Extraction Finished. \n");
    printf("[INFO] Starting Saving Models ... \n");


    stringstream ss;
	string id_str;
	ss << this->id;
	ss >> id_str;
	Model object_model= Model(this->id, this->name, raw_clouds, feature.extracted_slices, feature.key_points, feature.slices_span, feature.trimmed_slice_index, feature.height);
	// visualizePointCloud(object_model.slices);
    object_model.saveModel(id_str+"_"+this->name.substr(0, this->name.find(".pcd"))+".model");
    printf("[INFO] Models have been saved. \n");
    return new_model;
}
// void Modelling::visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb )
// {
// 	pcl::visualization::PCLVisualizer viewer("cloud");
// 	ColorHandlerT handle (cloud, rgb.getR(), rgb.getG(), rgb.getB());
// 	viewer.addPointCloud (cloud, handle, "cloud_visualization");
// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_visualization");
// 	// viewer.addCoordinateSystem (0.2, "sensor", 0);
//     viewer.addCoordinateSystem (0.1, 0, 0, 0);
// 	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
// 	// viewer.initCameraParameters ();
//   	while (!viewer.wasStopped ())
//   	{
//   		viewer.spinOnce ();
//   	}
// }
// void Modelling::visualizePointCloud ( PointCloudT::Ptr* cloud, RGB* rgb, int number)
// {
// 	pcl::visualization::PCLVisualizer viewer("cloud");
//     for (int i = 0; i < number; i++)
//     {
//         ColorHandlerT handle (cloud[i], rgb[i].getR(), rgb[i].getG(), rgb[i].getB());   
//         stringstream ss;
//         string str;
//         ss << i;
//         str = "cloud_"+ss.str();
//         viewer.addPointCloud (cloud[i], handle, str);
//         viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);
//     }
//     viewer.addCoordinateSystem (0.1, 0, 0, 0);
// 	viewer.setBackgroundColor(0., 0., 0., 0); 
// 	// viewer.initCameraParameters ();
//   	while (!viewer.wasStopped ())
//   	{
//   		viewer.spinOnce ();
//   	}
// }
void Modelling::poissonSampling(PointCloudT::Ptr &cloud, int ksearch, bool confidence, int degree, int depth, int ISO, bool manifold, bool polygon, float smooth, float scale, int solver, int min_depth )
{
    	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); 
		pcl::NormalEstimationOMP<pcl::PointXYZ , pcl::Normal> n ;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>) ;

	//	VoxelGridProperties voxel_properties = VoxelGridProperties(this->properties_filename, 1);
	//    voxelGridFilter(cloud, voxel_properties.size);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>) ;
		tree->setInputCloud(cloud) ;
        n.setNumberOfThreads (8);
		n.setInputCloud(cloud) ;
		n.setSearchMethod(tree) ;
		n.setKSearch(ksearch);
		n.compute(*normals);
        printf("[INFO] Normal Estimation Finished. \n");

		pcl::concatenateFields(*cloud , *normals , *cloud_with_normals) ;
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>) ;
		tree2->setInputCloud(cloud_with_normals) ;
		
        printf("[INFO] Starting Poisson Sampling... \n");
        pcl::Poisson<pcl::PointNormal> pn ;
		pn.setConfidence(confidence); 
		pn.setDegree(degree); 
		pn.setDepth(depth); 
		pn.setIsoDivide(ISO); 
		pn.setManifold(manifold); 
		pn.setOutputPolygons(polygon); 
		pn.setSamplesPerNode(smooth); 
		pn.setScale(scale);
		pn.setSolverDivide(solver); 
        // pn.setMinDepth(min_depth);

        printf("[INFO] Starting Reconstruction Process... \n");
		pn.setSearchMethod(tree2);
		pn.setInputCloud(cloud_with_normals);
		pcl::PolygonMesh mesh ;
		pn.performReconstruction(mesh);
		pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
        
}
void Modelling::visualizePointCloud ( vector<PointCloudT::Ptr> cloud)
{
    RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(255, 165,   0),
            RGB(255, 255,   0),
            RGB(  0, 255,   0),
            RGB(  0, 127, 255),
            RGB(  0,   0, 255),
            RGB(139,   0, 255),
            RGB(255, 255, 255),
            RGB(255, 255,   0)
        };
	pcl::visualization::PCLVisualizer viewer("cloud");
    for (int i = 0; i < cloud.size(); i++)
    {
        ColorHandlerT handle (cloud[i], rgbs[i%9].getR(), rgbs[i%9].getG(), rgbs[i%9].getB());   
        stringstream ss;
        string str;
        ss << i;
        str = "cloud_"+ss.str();
        viewer.addPointCloud (cloud[i], handle, str);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, str);
    }
    viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0., 0., 0., 0); 
	// viewer.initCameraParameters ();
  	while (!viewer.wasStopped ())
  	{
  		viewer.spinOnce ();
  	}
}
