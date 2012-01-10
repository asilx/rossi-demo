#include "featurizer.h"
#include "DataLogger.h"
namespace featurizer
{
  string
  Featurizer::getRandomString (int i, bool isZenith)
  {
    stringstream s;
    if (isZenith)
      s << "zenith" << i;
    else
      s << "azimuth" << i;
    cout << "jkj" << endl;
    string idmiz = s.str ();
    return idmiz;
  }

  // ++Onur

  void
  Featurizer::setFeatureVector (pcl::PointCloud<featurizer::NormalHistogram>::Ptr &azimuths, pcl::PointCloud<
      featurizer::NormalHistogram>::Ptr &zeniths)
  {
    int i;
    for (i = 0; i < 20; i++)
    {
      currentFeatureVector[i+6] = azimuths->points[0].histogram[i];
    }

    for (i = 20; i < 39; i++)
    {
      currentFeatureVector[i+6] = zeniths->points[0].histogram[i - 20];
    }
  }
  void Featurizer::setSizeAndPos(double posX, double posY, double posZ, double sizeX, double sizeY, double sizeZ)
  {
  	currentFeatureVector[0] = posX;
  	currentFeatureVector[1] = posY;
  	currentFeatureVector[2] = posZ;
  	currentFeatureVector[3] = sizeX;
  	currentFeatureVector[4] = sizeY;
  	currentFeatureVector[5] = sizeZ;
  }

  double*
  Featurizer::getCurrentFeatureVector ()
  {
    return &currentFeatureVector[0];
  }

  // --Onur

  void
  Featurizer::getNoOfClusters (const sensor_msgs::PointCloud2ConstPtr& cloud2)
  {
    //memcpy (&no_of_clusters, &cloud2->data[0], sizeof(int));
    no_of_clusters = 1;
  }

  double*
  Featurizer::extractAndGetFeatures (const sensor_msgs::PointCloud2& cloud2)
  {
    extractFeatures (cloud2);
    return getCurrentFeatureVector ();
  }

  vector<double*>
  Featurizer::extractAndVisualizeAll (const std::vector<sensor_msgs::PointCloud> &clusters)
  {
    vector<double*> featureCloud;
    no_of_clusters = clusters.size ();
    
    sensor_msgs::PointCloud2 convertedCloud;
/*
        sensor_msgs::convertPointCloudToPointCloud2 	( 	const sensor_msgs::PointCloud &  	input,
		sensor_msgs::PointCloud2 &  	output	 
	) 

*/
    featureCloud.resize (no_of_clusters);

    for (int i = 0; i < no_of_clusters; i++)
    {
    sensor_msgs::convertPointCloudToPointCloud2(clusters[i],convertedCloud);
      double* vect = new double[40];
      extractFeatures (convertedCloud, i, true);

	for(int j = 0; j < 40; j++)
	{
		vect[j] = currentFeatureVector[j];
	}
      featureCloud[i] = &vect[0];
      //featureLogger->logSingleData(&vect[0],i);
    }

    return featureCloud;
  }
  
  

  void
  Featurizer::extractFeatures (const sensor_msgs::PointCloud2& cloud2, int objIndex, bool visualization)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (cloud2, *cloud);
    cout << cloud->height << " " << cloud->width << " " << cloud->size () << endl;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    //cout << "Normal estimation is about to begin" << endl;

    /*pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
     ne.setMaxDepthChangeFactor(0.01f);
     ne.setNormalSmoothingSize(20.0f);
     ne.setInputCloud(cloud);
     cout << "tt3" << endl;
     ne.compute(*cloud_normals);*/

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (4);
    ne.setInputCloud (cloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.025);
    // Compute the features
    ne.compute (*cloud_normals);
    //cout << "Normal estimation was done" << endl;
    giveSurfaceHists (cloud_normals, azimuths, zeniths);

    setFeatureVector (azimuths, zeniths);

    if (visualization)//usePublications)
    {
      // The part where Visualization happens.
      sensor_msgs::PointCloud2 cloud_aligned;
      cloud_aligned.header.frame_id = "/base_link";
      cloud_aligned.height = 1;
      cloud_aligned.width = cloud->width / 1;
      cloud_aligned.fields.resize (7);
      cloud_aligned.fields[0].name = "x";
      cloud_aligned.fields[0].offset = 0;
      cloud_aligned.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[0].count = 1;
      cloud_aligned.fields[1].name = "y";
      cloud_aligned.fields[1].offset = 4;
      cloud_aligned.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[1].count = 1;
      cloud_aligned.fields[2].name = "z";
      cloud_aligned.fields[2].offset = 8;
      cloud_aligned.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[2].count = 1;
      cloud_aligned.fields[3].name = "normal_x";
      cloud_aligned.fields[3].offset = 12;
      cloud_aligned.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[3].count = 1;
      cloud_aligned.fields[4].name = "normal_y";
      cloud_aligned.fields[4].offset = 16;
      cloud_aligned.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[4].count = 1;
      cloud_aligned.fields[5].name = "normal_z";
      cloud_aligned.fields[5].offset = 20;
      cloud_aligned.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[5].count = 1;
      cloud_aligned.fields[6].name = "curvature";
      cloud_aligned.fields[6].offset = 24;
      cloud_aligned.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[6].count = 1;

      cloud_aligned.point_step = 28;
      cloud_aligned.row_step = cloud_aligned.point_step * cloud->width / 1;
      cloud_aligned.is_dense = true;

      cloud_aligned.data.resize ((cloud->width / 1) * 1 * 28);

      int zero = 0;
		int aq =1 ;
      for (int i = 0; i < (cloud->width / aq); i++)
      {
        for (int j = 0; j < 1; j++)
        {
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[0].offset],
                  &cloud->points[(cloud->width * 1 * j) + aq * i].x, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[1].offset],
                  &cloud->points[(cloud->width * 1 * j) + aq * i].y, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[2].offset],
                  &cloud->points[(cloud->width * 1 * j) + aq * i].z, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[3].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + aq * i].normal[0], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[4].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + aq * i].normal[1], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[5].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + aq * i].normal[2], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[6].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + aq * i].curvature, sizeof(float));
          zero++;
        }
      }

      pub->publish (cloud_aligned);

      int histog[20];// = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

      for(uint8_t i=0;i<20;i++)
        histog[i]=i*18;

      int azims[20];
      int zens[20];
      for (int i = 0; i < 20; i++)
      {
        azims[i] = azimuths->points[0].histogram[i];
        zens[i] = zeniths->points[0].histogram[i];
      }

      //cout << "Matlab executions" << endl;
      mxArray *x = mxCreateNumericMatrix (20, 1, mxINT32_CLASS, mxREAL);
      mxArray *zy = mxCreateNumericMatrix (20, 1, mxINT32_CLASS, mxREAL);
      mxArray *ay = mxCreateNumericMatrix (20, 1, mxINT32_CLASS, mxREAL);

      memcpy ((char *)mxGetPr (x), (char *)histog, 20 * sizeof(int));
      memcpy ((char *)mxGetPr (zy), (char *)zens, 20 * sizeof(int));
      memcpy ((char *)mxGetPr (ay), (char *)azims, 20 * sizeof(int));

      if(ep == NULL)
      {
        cout << "matlab engine could not be started" << endl;
      }
      else
      {
	      engPutVariable (ep, "x", x);
	      engPutVariable (ep, "zy", zy); // zenith
	      engPutVariable (ep, "ay", ay); // azimuth

	      stringstream s;
	      s << "subplot("<< no_of_clusters  <<", 2," << 2 * objIndex+1 << ")";
	      string cmd = s.str ();
	      stringstream title_s;
	      title_s << "title('zenith hist. for obj. " << objIndex << "')";
	      string titCMD = title_s.str();
	      stringstream s1;
	      s1 << "subplot("<< no_of_clusters  <<", 2," << 2 * objIndex+2 << ")";
	      string cmd2 = s1.str ();
	       stringstream title_s1;
	      title_s1 << "title('azimuth hist. for obj. " << objIndex << "')";
	      string titCMD1 = title_s1.str();     

	      /*int a = engEvalString (ep, cmd.c_str ());
	      int b = engEvalString (ep, "bar(x, zy);");
	      b = engEvalString(ep,titCMD.c_str());
	      int c = engEvalString (ep, cmd2.c_str ());
	      int d = engEvalString (ep, "bar(x, ay);");
	      b = engEvalString(ep,titCMD1.c_str());*/

	      //counter++;

	      //cout << "Visualization data for obj. " << objIndex << " sent!" << endl;
	}
    }

  }

  void
  Featurizer::extractFeatures (const sensor_msgs::PointCloud2& cloud2)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (cloud2, *cloud);
    //cout << cloud->height << " " << cloud->width << " " << cloud->size () << endl;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    /*pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
     ne.setMaxDepthChangeFactor(0.01f);
     ne.setNormalSmoothingSize(20.0f);
     ne.setInputCloud(cloud);
     cout << "tt3" << endl;
     ne.compute(*cloud_normals);*/

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (4);
    ne.setInputCloud (cloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.05);
    // Compute the features
    ne.compute (*cloud_normals);

    giveSurfaceHists (cloud_normals, azimuths, zeniths);

    setFeatureVector (azimuths, zeniths);

    if (usePublications)
    {
      // The part where Visualization happens.
      sensor_msgs::PointCloud2 cloud_aligned;
      cloud_aligned.header.frame_id = "/base_link";
      cloud_aligned.height = 1;
      cloud_aligned.width = cloud->width / 1;
      cloud_aligned.fields.resize (7);
      cloud_aligned.fields[0].name = "x";
      cloud_aligned.fields[0].offset = 0;
      cloud_aligned.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[0].count = 1;
      cloud_aligned.fields[1].name = "y";
      cloud_aligned.fields[1].offset = 4;
      cloud_aligned.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[1].count = 1;
      cloud_aligned.fields[2].name = "z";
      cloud_aligned.fields[2].offset = 8;
      cloud_aligned.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[2].count = 1;
      cloud_aligned.fields[3].name = "normal_x";
      cloud_aligned.fields[3].offset = 12;
      cloud_aligned.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[3].count = 1;
      cloud_aligned.fields[4].name = "normal_y";
      cloud_aligned.fields[4].offset = 16;
      cloud_aligned.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[4].count = 1;
      cloud_aligned.fields[5].name = "normal_z";
      cloud_aligned.fields[5].offset = 20;
      cloud_aligned.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[5].count = 1;
      cloud_aligned.fields[6].name = "curvature";
      cloud_aligned.fields[6].offset = 24;
      cloud_aligned.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
      cloud_aligned.fields[6].count = 1;

      cloud_aligned.point_step = 28;
      cloud_aligned.row_step = cloud_aligned.point_step * cloud->width / 1;
      cloud_aligned.is_dense = true;

      cloud_aligned.data.resize ((cloud->width / 1) * 1 * 28);

      int zero = 0;

      for (int i = 0; i < (cloud->width / 10); i++)
      {
        for (int j = 0; j < 1; j++)
        {
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[0].offset],
                  &cloud->points[(cloud->width * 1 * j) + 10 * i].x, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[1].offset],
                  &cloud->points[(cloud->width * 1 * j) + 10 * i].y, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[2].offset],
                  &cloud->points[(cloud->width * 1 * j) + 10 * i].z, sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[3].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + 10 * i].normal[0], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[4].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + 10 * i].normal[1], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[5].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + 10 * i].normal[2], sizeof(float));
          memcpy (&cloud_aligned.data[zero * cloud_aligned.point_step + cloud_aligned.fields[6].offset],
                  &cloud_normals->points[(cloud->width * 1 * j) + 10 * i].curvature, sizeof(float));
          zero++;
        }
      }
      //cout << "Iteration cycle is completed!" << endl;
      pub->publish (cloud_aligned);

      //		if(counter < 1)
      //		{
      //			phv->addFeatureHistogram(*azimuths, 150, getRandomString(0, false));
      //			phv->addFeatureHistogram(*zeniths, 150, getRandomString(0, true));
      //		}
      //		else
      //		{
      //			phv->updateFeatureHistogram(*azimuths, 150, getRandomString(0, false));
      //			phv->updateFeatureHistogram(*zeniths, 150, getRandomString(0, true));
      //		}

      counter++;
      //if(no_of_clusters == 0 || counter % 5 == 0)
      //{
      //		phv->spinOnce(true);

      //}
    }

  }

  void
  Featurizer::run (const sensor_msgs::PointCloud2& cloud2)
  {
    extractFeatures (cloud2);
  }

  void
  Featurizer::giveSurfaceHists (pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<
      featurizer::NormalHistogram>::Ptr azimuths, pcl::PointCloud<featurizer::NormalHistogram>::Ptr zeniths)
  {
    for (int i = 0; i < 20; i++)
    {
      azimuths->points[0].histogram[i] = 0;
      zeniths->points[0].histogram[i] = 0;
    }

    for (int x = 0; x < cloud_normals->height; x++)
      for (int y = 0; y < cloud_normals->width; y++)
      {

        float xdim = cloud_normals->points[x * cloud_normals->width + y].normal_x;
        float ydim = cloud_normals->points[x * cloud_normals->width + y].normal_y;
        float zdim = cloud_normals->points[x * cloud_normals->width + y].normal_z;

        float zenith = atan2 (sqrt (pow (xdim, 2) + pow (ydim, 2)), zdim);
        float azimuth = atan2 (ydim, xdim);

        if (zenith < 0)
          zenith += M_PI * 2;
        if (azimuth < 0)
          azimuth += M_PI * 2;
        azimuths->points[0].histogram[(int)floor (azimuth / (M_PI / 10.0))]++;
        zeniths->points[0].histogram[(int)floor (zenith / (M_PI / 10.0))]++;
      }

  }

  // ++Onur
bool Featurizer::isFinal()
{
	return objectInFinalCondition;
}

void Featurizer::setFinal(bool value)
{
	objectInFinalCondition = value;
}

void Featurizer::setInitVector()
{
	for(int i = 0 ; i < featureCount; i++)
	{
		initialConditionFeatureVector[i] = currentFeatureVector[i];
	}
}

double* Featurizer::getEffectVector()
{
	
	// Standard formulation for effect feature is <final_features> - <initial_features>
	for(int i = 0; i < featureCount; i++)
	{
		effectVector[i] = currentFeatureVector[i] - initialConditionFeatureVector[i];
	}
	
	return &effectVector[0];
}

  // Just a plain featurizer object
  Featurizer::Featurizer ()
  {
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr azimuths1 (new pcl::PointCloud<featurizer::NormalHistogram>);
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr zeniths1 (new pcl::PointCloud<featurizer::NormalHistogram>);
    azimuths = azimuths1;
    zeniths = zeniths1;

    azimuths->resize (1);
    zeniths->resize (1);

    no_of_clusters = 0; //?

    usePublications = false;
    ep = engOpen(NULL);

    if(ep == NULL)
    {
      cout << "Constructor: engOpen failed" <<endl;
    }
    else
    {
      cout << "Opened the engine!" << endl;
    }
    
// featureCount = 40; // TODO: revise for adding pos & size

    featureLogger = new DataLogger("object","./",featureCount);
    effectLogger  = new DataLogger("effect","./",featureCount);
  }

  // --Onur


  Featurizer::Featurizer (ros::NodeHandle* nh3)
  {
    //nh3 = &nh;
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr azimuths1 (new pcl::PointCloud<featurizer::NormalHistogram>);
    pcl::PointCloud<featurizer::NormalHistogram>::Ptr zeniths1 (new pcl::PointCloud<featurizer::NormalHistogram>);
    azimuths = azimuths1;
    zeniths = zeniths1;

    azimuths->resize (1);
    zeniths->resize (1);

    pub = new ros::Publisher ();
    pub2 = new ros::Publisher ();
    pub3 = new ros::Publisher ();
    sub = new ros::Subscriber ();
    sub2 = new ros::Subscriber ();
    *pub = nh3->advertise<sensor_msgs::PointCloud2> ("/points2_out", 1);
    //*pub2 = nh3->advertise<pcl::PointCloud<pcl::FPFHSignature33> >("/feat_out", 1);
    //*pub3 = nh3->advertise<sensor_msgs::PointCloud2>("/points3_out", 1);
    *sub = nh3->subscribe ("/clusters_out", 1000, &Featurizer::run, this);
    *sub2 = nh3->subscribe ("/no_out", 1000, &Featurizer::getNoOfClusters, this);
   // phv = new pcl::visualization::PCLHistogramVisualizer;
    counter = 0;
    no_of_clusters = 0;

    ep = engOpen(NULL);
    if(ep == NULL)
    {
      cout << "Constructor: engOpen failed" <<endl;
    }
    else
    {
      cout << "Opened the engine!" << endl;
    }
    //usePublications = true;
    usePublications = false;
// featureCount = 40; // TODO: revise for adding pos & size

    featureLogger = new DataLogger("object","./",featureCount);
    effectLogger  = new DataLogger("effect","./",featureCount);
  }
}
