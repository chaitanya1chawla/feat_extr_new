//
// Created by chaitanya11 on 31.05.23.
//

#include <iostream>
#include <PerceptionData/containers/MarkerDetectionContainer.h>
#include <PerceptionData/containers/ObjectDetectionContainer.h>
#include <PerceptionData/containers/ObjectTagModelDetectionContainer.h>
#include <PerceptionData/containers/PersonDetectionContainer.h>
#include <PerceptionData/containers/SurfaceDetectionContainer.h>
#include <PerceptionData/PerceptionSequence.h>
#include <VisualPerception/utils/Perception.h>
#include <VisualPerception/utils/utils.h>
#include <VisualPerception/inputs/data/ColorData.h>
#include <VisualPerception/inputs/data/DepthData.h>
#include <VisualPerception/inputs/data/PointCloudData.h>

using namespace AndreiUtils;
using namespace cv;
using namespace nlohmann;
using namespace PerceptionData;
using namespace std;
using namespace VisualPerception;

void getPerceptionOutputs() {
    Perception p("Perception");
    cout << "Before initialize" << endl;
    p.initialize();

    cout << "Before perceptionInitialization" << endl;
    if (!p.perceptionInitialization()) {
        return;
    }

    // <{outputImage: outputImage, inputImageClone: inputImageClone, originalDepth: originalDepth}, depthIntrinsics, {openpose: skeletons, darknet: yoloDetections}>
    VisualPerceptionOutputData output;

    map<string, PerceptionSequence> sequences;

    int count = 0, exit = 0;
    //*
    namedWindow("Depth - Input", WINDOW_NORMAL);
    resizeWindow("Depth - Input", 1352, 1013);
    namedWindow("Color - Input", WINDOW_NORMAL);
    resizeWindow("Color - Input", 1352, 1013);
    namedWindow("Color - Output", WINDOW_NORMAL);
    resizeWindow("Color - Output", 1352, 1013);
    //*/
    PointCloudData::ContentType const *pointCloudData;
    cv::Mat const *depthData, *colorData, *outputColorData;
    for (; exit == 0; count++) {
        // cout << "In while at " << count << endl;
        if (!p.perceptionIteration()) {
            cout << "Perception Iteration returned false" << endl;
            exit = 1;
        }
        p.getOutput(output);
        vector<string> devices = output.getDevicesList();
        for (auto const &device: devices) {
            PerceptionOutput perceptionOutput(count);
            PerceptionDataContainer *deviceOutputContainer;
            if (output.getDeviceDataIfContains<MarkerDetectionContainer>(deviceOutputContainer, device)) {
                perceptionOutput.set(*(MarkerDetectionContainer *) deviceOutputContainer);
            }
            if (output.getDeviceDataIfContains<ObjectDetectionContainer>(deviceOutputContainer, device)) {
                perceptionOutput.set(*(ObjectDetectionContainer *) deviceOutputContainer);
            }
            if (output.getDeviceDataIfContains<ObjectTagModelDetectionContainer>(deviceOutputContainer, device)) {
                perceptionOutput.set(*(ObjectTagModelDetectionContainer *) deviceOutputContainer);
            }
            if (output.getDeviceDataIfContains<PersonDetectionContainer>(deviceOutputContainer, device)) {
                perceptionOutput.set(*(PersonDetectionContainer *) deviceOutputContainer);
            }
            if (output.getDeviceDataIfContains<SurfaceDetectionContainer>(deviceOutputContainer, device)) {
                perceptionOutput.set(*(SurfaceDetectionContainer *) deviceOutputContainer);
            }
            if (!mapContains(sequences, device)) {
                sequences[device];
            }
            PerceptionSequence &seq = mapGet(sequences, device);
            seq.addPerceptionOutput(perceptionOutput, count);
        }

        //*
        if (output.getInputDataIfContains<PointCloudData>(pointCloudData)) {
            cout << "InputCloud: " << (uint32_t *) (*pointCloudData)->data() << endl;
        }
        //*/
        if (output.getInputDataIfContains<DepthData>(depthData)) {
            // cout << "InputDepth: " << (uint32_t *) depthData->data << endl;
            imshow("Depth - Input", *depthData);
        }
        if (output.getInputDataIfContains<ColorData>(colorData)) {
            // cout << "InputColor: " << (uint32_t *) colorData->data << endl;
            imshow("Color - Input", *colorData);
        }
        if (output.getOutputDataIfContains<ColorData>(outputColorData)) {
            // cout << "OutputColor: " << (uint32_t *) outputColorData->data << endl;
            imshow("Color - Output", *outputColorData);
        }
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            cout << "Manual stop" << endl;
            exit = 1;
        }
    }

    destroyAllWindows();
    p.finish();
}

int main() {
    cout << "Hello, World!" << endl;
    setConfigurationParametersDirectory("../config/");

    initializeMarkerDetectionLibForVisualPerception();
    initializeDarknetForVisualPerception();
    initializeOpenposeForVisualPerception();
    initializePclForVisualPerception();
    initializePclRealsenseForVisualPerception();
    initializeRealsenseForVisualPerception();

    getPerceptionOutputs();

    return 0;
}