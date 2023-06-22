//
// Created by Andrei on 31.05.23.
//

#include <algorithm>            // min, max
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>
#include <utility>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>

#include <AndreiUtils/classes/camera/ImageCaptureParametersWithIntrinsics.h>
#include <AndreiUtils/utilsJson.h>
#include <AndreiUtils/utilsJsonEigen.hpp>
#include <AndreiUtils/utilsOpenCV.hpp>
#include <AndreiUtils/utilsRealsense.h>
#include <AndreiUtils/utilsOpenCVRealsense.h>
#include <AndreiUtils/utilsTime.h>
#include <MarkerDetectionLib/AprilTagDetectorWithOpenCV.h>
#include <MarkerDetectionLib/utils/utils.h>
#include <chrono>

#include "example.hpp"

using namespace AndreiUtils;
using namespace cv;
using namespace Eigen;
using namespace MarkerDetectionLib;
using namespace pcl;
using namespace rs2;
using namespace std;
using json = nlohmann::json;

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
              ml(false), offset_x(0.0f), offset_y(0.0f) {}

    double yaw, pitch, last_x, last_y;
    bool ml;
    float offset_x, offset_y;
};

float3 colors[]{
        {0.8f,  0.1f, 0.3f},
        {0.1f,  0.9f, 0.5f},
        {0.2f,  0.5f, 0.8f},
        {0.3f,  0.9f, 0.8f},
        {0.76f, 0.9f, 0.4f},
        {0.8f,  0.3f, 0.9f},
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Helper functions
void register_glfw_callbacks(window &app, state &app_state);

void draw_pointcloud(window &app, state &app_state, std::vector<pcl_ptr> const &points, Eigen::MatrixXd &allAxes,
                     Posed &cameraToGround);

void printPoint(pcl::PointXYZ const &point) {
    cout << point.x << " " << point.y << " " << point.z << endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(rs2::points const &points) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    auto const *checkPtr = reinterpret_cast<float const *>(ptr);
    for (auto &p: cloud->points) {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        assert(ptr->x == checkPtr[0]);
        assert(ptr->y == checkPtr[1]);
        assert(ptr->z == checkPtr[2]);
        ptr++;
        checkPtr += 3;
    }

    return cloud;
}

void testSurfaceExtraction() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 15;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // Generate the data
    for (auto &point: *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1.0;
    }

    // Set a few outliers
    (*cloud)[0].z = 2.0;
    (*cloud)[3].z = -2.0;
    (*cloud)[6].z = 4.0;

    cout << "Point cloud data: " << cloud->size() << " points" << endl;
    for (const auto &point: *cloud) {
        printPoint(point);
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return;
    }

    cout << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
         << coefficients->values[2] << " " << coefficients->values[3] << endl;

    cout << "Model inliers: " << inliers->indices.size() << endl;
    for (const auto &idx: inliers->indices) {
        cout << idx << ": ";
        printPoint(cloud->points[idx]);
    }
}

class Surface {
public:
    using PCL = pcl::PointCloud<pcl::PointXYZ>;

    explicit Surface(PCL surfacePoints, pcl::ModelCoefficients coefficients) :
            points(std::move(surfacePoints)), coefficients(std::move(coefficients)) {
        ctr = 0;
        ctr = this->points.size();

    }

    PCL const &getPoints() const {
        return this->points;
    }

    PCL &getPoints() {
        return this->points;
    }

    int getNumberOfPoints() const {
        return ctr;
    }

protected:
    PCL points;
    pcl::ModelCoefficients coefficients;
    int ctr;
};


class SurfaceExtractor {
public:
    using P = pcl::PointXYZ;
    using PCL = pcl::PointCloud<P>;

    SurfaceExtractor(float leafSize, int modelType, int methodType, int maxIterations = 1000,
                     double distanceThreshold = 0.01, bool optimize = true) :
            sor(), cloud_filtered(new PCL), cloud_p(new PCL), cloud_f(new PCL),
            coefficients(new pcl::ModelCoefficients()), inliers(new pcl::PointIndices()) {
        this->sor.setLeafSize(leafSize, leafSize, leafSize);

        // Optional
        this->seg.setOptimizeCoefficients(optimize);
        // Mandatory
        this->seg.setModelType(modelType);
        this->seg.setMethodType(methodType);
        this->seg.setMaxIterations(maxIterations);
        this->seg.setDistanceThreshold(distanceThreshold);
    }

    PCL::Ptr filterPointCloud(PCL::Ptr const &cloud, bool verbose = false) {
        if (verbose) {
            cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << endl;
        }
        this->sor.setInputCloud(cloud);
        this->sor.filter(*this->cloud_filtered);
        if (verbose) {
            cout << "PointCloud after filtering: " << this->cloud_filtered->width * this->cloud_filtered->height
                 << " data points." << endl;
        }
        return this->cloud_filtered;
    }

    std::vector<Surface> extractSurfaces(PCL::Ptr const &cloud, double nrPointsThreshold = 0.2, bool verbose = false) {
        this->filterPointCloud(cloud, verbose);

        // Write the downsampled version to disk
        // pcl::PCDWriter writer;
        // writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

        int i = 0, nr_points = (int) this->cloud_filtered->size();
        cout << "Initial number of points: " << nr_points << endl;
        vector<Surface> surfaces;
        // While 30% of the original cloud is still there
        while ((double) this->cloud_filtered->size() > nrPointsThreshold * nr_points) {
            // Segment the largest planar component from the remaining cloud
            this->seg.setInputCloud(this->cloud_filtered);
            this->seg.segment(*this->inliers, *this->coefficients);
            if (this->inliers->indices.empty()) {
                cout << "Could not estimate a planar model for the given dataset." << endl;
                break;
            }

            // Extract the inliers
            this->extract.setInputCloud(this->cloud_filtered);
            this->extract.setIndices(this->inliers);
            this->extract.setNegative(false);  // set to get inliers
            this->extract.filter(*this->cloud_p);
            if (true || verbose) {
                cout << "PointCloud representing the planar component: " << this->cloud_p->width * this->cloud_p->height
                     << " data points." << endl;
            }
            surfaces.emplace_back(*this->cloud_p, *this->coefficients);

            // stringstream ss;
            // ss << "table_scene_lms400_plane_" << i << ".pcd";
            // writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

            // Use the filtering object
            this->extract.setNegative(true);  // set to get outliers
            this->extract.filter(*this->cloud_f);
            this->cloud_filtered.swap(this->cloud_f);
            if (verbose) {
                cout << "filtered cloud point size: " << this->cloud_filtered->size() << endl;
            }
            i++;
        }

        return surfaces;
    }

protected:
    PCL::Ptr cloud_filtered, cloud_p, cloud_f;

    // Create the filtering object
    pcl::ExtractIndices<P> extract;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<P> sor;

    // Create the segmentation object
    pcl::SACSegmentation<P> seg;
};

bool findGroundInImage(Posed &groundPose, rs2::video_frame &color, rs2::depth_frame &depth,
                       AprilTagDetectorWithOpenCV &detector,
                       int const &groundMarkerId) {
    ImageCaptureParametersWithIntrinsics intrinsics;
    AndreiUtils::convertRealsenseIntrinsicsToCameraIntrinsicParameters(
            color.get_profile().as<rs2::video_stream_profile>().get_intrinsics(),
            intrinsics.size, intrinsics.intrinsics);
    cv::Mat colorImage = convertFrameToMat(color);
    cv::Mat depthImage = convertDepthFrameToMeters(depth);
    cv::resize(depthImage, depthImage, {intrinsics.size.w, intrinsics.size.h});
    auto getDepth = getMatrixElementAccessorWithReferenceToData<double, float>(depthImage);
    vector<MarkerDetection> detections = detector.detectMarkers(colorImage, intrinsics.intrinsics, &getDepth,
                                                                intrinsics.size);
    bool foundGround = false;
    for (auto const &detection: detections) {
        if (detection.getId() == groundMarkerId) {
            foundGround = true;
            groundPose = detection.getPoseTagToCamera();
        }
    }
    return foundGround;
}

void realsensePointCloud() {

    // Helper functions
    try {
        vector<nlohmann::json> tracking;

        SurfaceExtractor surfaceExtractor(0.015f, pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 1000, 0.02);

        // Create a simple OpenGL window for rendering:
        window app(1280, 720, "RealSense PCL Pointcloud Example");
        // Construct an object to manage view state
        state app_state;
        // register callbacks to allow manipulation of the pointcloud
        register_glfw_callbacks(app, app_state);

        // Declare pointcloud object, for calculating pointclouds and texture mappings
        rs2::pointcloud pc;
        // We want the points object to be persistent so we can display the last cloud when a frame drops
        rs2::points points;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        pipe.start();

        pcl_ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 3.0);

        Eigen::Matrix3d B;

        AprilTagDetectorWithOpenCV detector("AprilTagDetector");
        int groundMarkerId = 209;

        cv::Mat displayImage(100, 100, CV_8UC3);
        int frameNumber = 0;

        while (app) {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            auto color = frames.get_color_frame();
            if (!color) {
                // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
                color = frames.get_infrared_frame();
            }

            // Tell pointcloud object to map to this color frame
            pc.map_to(color);

            auto depth = frames.get_depth_frame();

            Posed groundPoseRelativeToCamera, cameraToGround;
            if (!findGroundInImage(groundPoseRelativeToCamera, color, depth, detector, groundMarkerId)) {
                continue;
            }
            auto t = groundPoseRelativeToCamera.getRotationAsMatrix();
            cout << "Rotation Matrix = " << endl << t << endl;

            cameraToGround = groundPoseRelativeToCamera.inverse();

            // Generate the pointcloud and texture mappings
            points = pc.calculate(depth);
            auto pclPoints = points_to_pcl(points);

            pass.setInputCloud(pclPoints);
            pass.filter(*cloudFiltered);

            std::vector<pcl_ptr> layers;
            // layers.push_back(pclPoints);
            // layers.push_back(cloudFiltered);

            auto surfaces = surfaceExtractor.extractSurfaces(cloudFiltered, 0.15);
            cout << "Found " << surfaces.size() << " surfaces in the point cloud!" << endl;

            json frame;
            frame["index"] = frameNumber++;
            frame["numberOfSurfaces"] = surfaces.size();

            int surfaceCounter = 0;
            vector<json> surfacesData(surfaces.size());

            // used to send the axes and origin to the drawPointCloud function
            Eigen::MatrixXd allAxes;
            // each column represents x,y,z, and origin
            // first 3 rows represent data of first surfaces, second 3 rows that of second surface, and so on
            allAxes.resize(3 * surfaces.size(), 4);

            for (auto const &surface: surfaces) {

                json singleSurface;
                layers.emplace_back(new Surface::PCL(surface.getPoints()));
                int j = 0;

                // A is the matrix having coordinates of all points
                Eigen::MatrixXd A;
                cout << "No. of Points = " << surface.getNumberOfPoints() << endl;
                A.resize(surface.getNumberOfPoints() + 1, 3);
                for (auto Pt: surface.getPoints()) {
                    A.row(j) = cameraToGround.transform(Eigen::Vector3d(Pt.x, Pt.y, Pt.z));
                    j++;
                }

                // origin = A.rowwise().mean();
                Eigen::Vector3d origin = Eigen::Vector3d(
                        A.col(0).mean(),
                        A.col(1).mean(),
                        A.col(2).mean());

                // subtracting mean of all points from the entire matrix - to bring surface to its origin
                A = A.rowwise() - origin.transpose();

                Eigen::BDCSVD<Eigen::MatrixXd> svd;
                //cout<<"A matrix = "<<A<<endl;
                svd.compute(A, Eigen::ComputeFullV);

                // The 3 columns of V represent the 3 axes x,y,z respectively of the new surface
                auto v = svd.matrixV();
                cout << "V matrix for " << surfaceCounter << " : " << endl << v << endl;

                // transformed PointCloud wrt to the new axes from V
                auto newCoordinates = v * A.transpose(); //(Eigen::placeholders::all,vector<int> {0,1})

                vector<double> x_range{
                        newCoordinates.rowwise().minCoeff()(0),
                        newCoordinates.rowwise().maxCoeff()(0)};
                vector<double> y_range{
                        newCoordinates.rowwise().minCoeff()(1),
                        newCoordinates.rowwise().maxCoeff()(1)};

                Eigen::Vector3d x_axis = Eigen::Vector3d{v.col(0)}.normalized();
                Eigen::Vector3d y_axis = Eigen::Vector3d{v.col(1)}.normalized();
                Eigen::Vector3d z_axis = Eigen::Vector3d{v.col(0)}.cross(Eigen::Vector3d{v.col(1)}).normalized();

                allAxes.block(surfaceCounter * 3, 0, 3, 1) = x_axis;
                allAxes.block(surfaceCounter * 3, 1, 3, 1) = y_axis;
                allAxes.block(surfaceCounter * 3, 2, 3, 1) = z_axis;
                allAxes.block(surfaceCounter * 3, 3, 3, 1) = origin;

                singleSurface["origin"] = origin;
                singleSurface["surfaceNumber"] = surfaceCounter;
                singleSurface["xAxis"] = x_axis;
                singleSurface["yAxis"] = y_axis;
                singleSurface["zAxis"] = z_axis;
                singleSurface["xRange"] = x_range;
                singleSurface["yRange"] = y_range;

                cout << "Max coordinates in new dimension -" << endl << "x = " << x_range[1] << " and y = "
                     << y_range[1] << endl;
                cout << "Min coordinates in new dimension -" << endl << "x = " << x_range[0] << " and y = "
                     << y_range[0] << endl;

                surfacesData[surfaceCounter++] = singleSurface;
            }
            frame["surfacesData"] = surfacesData;
            draw_pointcloud(app, app_state, layers, allAxes, groundPoseRelativeToCamera);
            tracking.push_back(frame);
        }

        string timeStr = convertChronoToStringWithSubsecondsCustomJoin(SystemClock::now(), "_");
        string demonstrationFile = "../data/surfaceData_" + timeStr + ".json";
        writeJsonFile(demonstrationFile, tracking);
    }
    catch (const rs2::error &e) {
        cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args()
             << "):\n    " << e.what() << endl;
    }
    catch (const exception &e) {
        cerr << e.what() << endl;
    }
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
void register_glfw_callbacks(window &app, state &app_state) {
    app.on_left_mouse = [&](bool pressed) {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset) {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y) {
        if (app_state.ml) {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key) {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0;
            app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window &app, state &app_state, const std::vector<pcl_ptr> &points, Eigen::MatrixXd &allAxes,
                     Posed &cameraToGround) {
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y * 0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;
    int counter = 0;


    for (auto &&pc: points) {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];
        // cout << "Color of surface"<<counter++<<" = "<< <<endl;
        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (auto &p: pc->points) {
            if (p.z != 0) {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    int numberOfSurfaces = allAxes.size() / (3 * 3);
    int ctr = 0;
    cout << "allAxes = " << endl << allAxes << endl;
    glBegin(GL_LINES);


    while (ctr < numberOfSurfaces - 1) {

        if (allAxes.maxCoeff() > 100 || allAxes.minCoeff() < -100) {
            ctr++;
            continue;
        }

        Posed q = Posed::identity().addTranslation(allAxes.block(ctr*3, 3, 3, 1));
        q = cameraToGround * q;
        auto t = q.getTranslation(); // getting origin wrt to camera

        q = Posed::identity().addTranslation(allAxes.block(ctr*3, 0, 3, 1) + allAxes.block(ctr*3, 3, 3, 1));
        q = cameraToGround * q;
        auto x_axis = q.getTranslation(); // getting x_axis wrt to camera

        q = Posed::identity().addTranslation(allAxes.block(ctr*3, 1, 3, 1) + allAxes.block(ctr*3, 3, 3, 1));
        q = cameraToGround * q;
        auto y_axis = q.getTranslation(); // getting y_axis wrt to camera

        q = Posed::identity().addTranslation(allAxes.block(ctr*3, 2, 3, 1)+ allAxes.block(ctr*3, 3, 3, 1));
        q = cameraToGround * q;
        auto z_axis = q.getTranslation(); // getting z_axis wrt to camera

        // x axis
        glColor3f(0.0f, 0.0f, 1.0f); // blue
        glVertex3f(t.x(), t.y(), t.z());
        glVertex3f(x_axis.x(), x_axis.y(), x_axis.z());

        // y axis
        glColor3f(0.0f, 1.0f, 0.0f); // green
        glVertex3f(t.x(), t.y(), t.z());
        glVertex3f(y_axis.x(), y_axis.y(), y_axis.z());

        // z axis
        glColor3f(1.0f, 0.0f, 0.0f); // red
        glVertex3f(t.x(), t.y(), t.z());
        glVertex3f(z_axis.x(), z_axis.y(), z_axis.z());

        ctr++;
    }
    glEnd();

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

int main() {
    cout << "Hello, World!" << endl;

    auto config = readJsonFile("../config/surfaceExtractionArguments.json");
    MarkerDetectionLib::setConfigurationParameters(
            ConfigurationParameters(config.at("MarkerDetectionLib"), "MarkerDetectionLib"));

    // testSurfaceExtraction();
    realsensePointCloud();

    return 0;
}
