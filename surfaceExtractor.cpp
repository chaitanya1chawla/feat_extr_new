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

#include "example.hpp"

using namespace cv;
using namespace pcl;
using namespace rs2;
using namespace std;

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

void draw_pointcloud(window &app, state &app_state, std::vector<pcl_ptr> const &points);

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
        // TODO: compute the surface range here!
    }

    PCL const &getPoints() const {
        return this->points;
    }

    PCL &getPoints() {
        return this->points;
    }

protected:
    PCL points;
    pcl::ModelCoefficients coefficients;
    std::pair<double, double> rangeX, rangeY;
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

void realsensePointCloud() {
    // Helper functions
    try
    {
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

        cv::Mat displayImage(100, 100, CV_8UC3);
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
            for (auto const &surface: surfaces) {
                layers.emplace_back(new Surface::PCL(surface.getPoints()));
            }

            draw_pointcloud(app, app_state, layers);
        }
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
void draw_pointcloud(window &app, state &app_state, const std::vector<pcl_ptr> &points) {
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

    for (auto &&pc: points) {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

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

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

int main() {
    cout << "Hello, World!" << endl;

    // testSurfaceExtraction();
    realsensePointCloud();

    return 0;
}
