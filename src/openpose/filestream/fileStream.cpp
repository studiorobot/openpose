#include <openpose/filestream/fileStream.hpp>
#include <iostream>
#include <fstream> // std::ifstream, std::ofstream
#include <opencv2/highgui/highgui.hpp> // cv::imread
#include <openpose/utilities/fastMath.hpp>
#include <openpose/utilities/fileSystem.hpp>
#include <openpose/utilities/string.hpp>
#include <openpose/filestream/jsonOfstream.hpp>

#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include "example.h"


#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//added by MJ
#include <iostream>
#include <fstream>
#include <map>
#include <utility>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH           640               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          0                 // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


namespace op
{
    // Private class (on *.cpp)
    const auto errorMessage = "Json format only implemented in OpenCV for versions >= 3.0. Check savePoseJson"
                              " instead.";

    std::string getFullName(const std::string& fileNameNoExtension, const DataFormat dataFormat)
    {
        return fileNameNoExtension + "." + dataFormatToString(dataFormat);
    }

    void addKeypointsToJson(
        JsonOfstream& jsonOfstream, const std::vector<std::pair<Array<float>, std::string>>& keypointVector)
    {
        try
        {
            // Sanity check
            for (const auto& keypointPair : keypointVector)
                if (!keypointPair.first.empty() && keypointPair.first.getNumberDimensions() != 3
                    && keypointPair.first.getNumberDimensions() != 1)
                    error("keypointVector.getNumberDimensions() != 1 && != 3.", __LINE__, __FUNCTION__, __FILE__);
            // Add people keypoints
            jsonOfstream.key("people");
            jsonOfstream.arrayOpen();
            // Ger max numberPeople
            auto numberPeople = 0;
            for (auto vectorIndex = 0u ; vectorIndex < keypointVector.size() ; vectorIndex++)
                numberPeople = fastMax(numberPeople, keypointVector[vectorIndex].first.getSize(0));
            for (auto person = 0 ; person < numberPeople ; person++)
            {
                jsonOfstream.objectOpen();
                for (auto vectorIndex = 0u ; vectorIndex < keypointVector.size() ; vectorIndex++)
                {
                    const auto& keypoints = keypointVector[vectorIndex].first;
                    const auto& keypointName = keypointVector[vectorIndex].second;
                    const auto numberElementsPerRaw = keypoints.getSize(1) * keypoints.getSize(2);
                    jsonOfstream.key(keypointName);
                    jsonOfstream.arrayOpen();
                    // Body parts
                    if (numberElementsPerRaw > 0)
                    {
                        const auto finalIndex = person*numberElementsPerRaw;
                        for (auto element = 0 ; element < numberElementsPerRaw - 1 ; element++)
                        {
                            jsonOfstream.plainText(keypoints[finalIndex + element]);
                            jsonOfstream.comma();
                        }
                        // Last element (no comma)
                        jsonOfstream.plainText(keypoints[finalIndex + numberElementsPerRaw - 1]);
                    }
                    // Close array
                    jsonOfstream.arrayClose();
                    if (vectorIndex < keypointVector.size()-1)
                        jsonOfstream.comma();
                }
                jsonOfstream.objectClose();
                if (person < numberPeople-1)
                {
                    jsonOfstream.comma();
                    jsonOfstream.enter();
                }
            }
            // Close bodies array
            jsonOfstream.arrayClose();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void addCandidatesToJson(
        JsonOfstream& jsonOfstream, const std::vector<std::vector<std::array<float,3>>>& candidates)
    {
        try
        {
            // Add body part candidates
            jsonOfstream.key("part_candidates");
            jsonOfstream.arrayOpen();
            // Ger max numberParts
            const auto numberParts = candidates.size();
            jsonOfstream.objectOpen();
            for (auto part = 0u ; part < numberParts ; part++)
            {
                // Open array
                jsonOfstream.key(std::to_string(part));
                jsonOfstream.arrayOpen();
                // Iterate over part candidates
                const auto& partCandidates = candidates[part];
                const auto numberPartCandidates = partCandidates.size();
                // Body part candidates
                for (auto bodyPart = 0u ; bodyPart < numberPartCandidates ; bodyPart++)
                {
                    const auto& candidate = partCandidates[bodyPart];
                    jsonOfstream.plainText(candidate[0]);
                    jsonOfstream.comma();
                    jsonOfstream.plainText(candidate[1]);
                    jsonOfstream.comma();
                    jsonOfstream.plainText(candidate[2]);
                    if (bodyPart < numberPartCandidates-1)
                        jsonOfstream.comma();
                }
                jsonOfstream.arrayClose();
                if (part < numberParts-1)
                    jsonOfstream.comma();
            }
            jsonOfstream.objectClose();
            // Close array
            jsonOfstream.arrayClose();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }





    // Public classes (on *.hpp)
    std::string dataFormatToString(const DataFormat dataFormat)
    {
        try
        {
            if (dataFormat == DataFormat::Json)
                return "json";
            else if (dataFormat == DataFormat::Xml)
                return "xml";
            else if (dataFormat == DataFormat::Yaml)
                return "yaml";
            else if (dataFormat == DataFormat::Yml)
                return "yml";
            else
            {
                error("Undefined DataFormat.", __LINE__, __FUNCTION__, __FILE__);
                return "";
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return "";
        }
    }

    DataFormat stringToDataFormat(const std::string& dataFormat)
    {
        try
        {
            if (dataFormat == "json")
                return DataFormat::Json;
            else if (dataFormat == "xml")
                return DataFormat::Xml;
            else if (dataFormat == "yaml")
                return DataFormat::Yaml;
            else if (dataFormat == "yml")
                return DataFormat::Yml;
            else
            {
                error("String does not correspond to any known format (json, xml, yaml, yml)",
                      __LINE__, __FUNCTION__, __FILE__);
                return DataFormat::Json;
            }
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return DataFormat::Json;
        }
    }

    void saveFloatArray(const Array<float>& array, const std::string& fullFilePath)
    {
        try
        {
            // Open file
            std::ofstream outputFile;
            outputFile.open(fullFilePath, std::ios::binary);
            // Save #dimensions
            const auto numberDimensions = (float)(array.getNumberDimensions());
            outputFile.write((char*)&numberDimensions, sizeof(float));
            // Save dimensions
            for (const auto& sizeI : array.getSize())
            {
                const float sizeIFloat = (float) sizeI;
                outputFile.write((char*)&sizeIFloat, sizeof(float));
            }
            // Save each value
            outputFile.write((char*)&array[0], array.getVolume() * sizeof(float));
            // Close file
            outputFile.close();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void saveData(const std::vector<Matrix>& opMats, const std::vector<std::string>& cvMatNames,
                  const std::string& fileNameNoExtension, const DataFormat dataFormat)
    {
        try
        {
            OP_OP2CVVECTORMAT(cvMats, opMats)
            // Sanity checks
            if (dataFormat == DataFormat::Json && CV_MAJOR_VERSION < 3)
                error(errorMessage, __LINE__, __FUNCTION__, __FILE__);
            if (cvMats.size() != cvMatNames.size())
                error("cvMats.size() != cvMatNames.size() (" + std::to_string(cvMats.size())
                      + " vs. " + std::to_string(cvMatNames.size()) + ")", __LINE__, __FUNCTION__, __FILE__);
            // Save cv::Mat data
            cv::FileStorage fileStorage{getFullName(fileNameNoExtension, dataFormat), cv::FileStorage::WRITE};
            for (auto i = 0u ; i < cvMats.size() ; i++)
                fileStorage << cvMatNames[i] << (cvMats[i].empty() ? cv::Mat() : cvMats[i]);
            // Release file
            fileStorage.release();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void saveData(const Matrix& opMat, const std::string cvMatName, const std::string& fileNameNoExtension,
                  const DataFormat dataFormat)
    {
        try
        {
            saveData(std::vector<Matrix>{opMat}, std::vector<std::string>{cvMatName}, fileNameNoExtension,
                     dataFormat);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    std::vector<Matrix> loadData(const std::vector<std::string>& cvMatNames, const std::string& fileNameNoExtension,
                                  const DataFormat dataFormat)
    {
        try
        {
            // Sanity check
            if (dataFormat == DataFormat::Json && CV_MAJOR_VERSION < 3)
                error(errorMessage, __LINE__, __FUNCTION__, __FILE__);
            // File name
            const auto fileName = getFullName(fileNameNoExtension, dataFormat);
            // Sanity check
            if (!existFile(fileName))
                error("File to be read does not exist: " + fileName + ".", __LINE__, __FUNCTION__, __FILE__);
            // Read file
            cv::FileStorage fileStorage{fileName, cv::FileStorage::READ};
            std::vector<cv::Mat> cvMats(cvMatNames.size());
            for (auto i = 0u ; i < cvMats.size() ; i++)
                fileStorage[cvMatNames[i]] >> cvMats[i];
            fileStorage.release();
            OP_CV2OPVECTORMAT(opMats, cvMats)
            return opMats;
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return {};
        }
    }

    Matrix loadData(const std::string& cvMatName, const std::string& fileNameNoExtension, const DataFormat dataFormat)
    {
        try
        {
            return OP_CV2OPMAT(loadData(std::vector<std::string>{cvMatName}, fileNameNoExtension, dataFormat)[0]);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return Matrix();
        }
    }

    void savePeopleJson(
        const Array<float>& keypoints, const std::vector<std::vector<std::array<float,3>>>& candidates,
        const std::string& keypointName, const std::string& fileName, const bool humanReadable)
    {
        try
        {
            savePeopleJson(
                std::vector<std::pair<Array<float>, std::string>>{std::make_pair(keypoints, keypointName)},
                candidates, fileName, humanReadable
            );
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void savePeopleJson(
        const std::vector<std::pair<Array<float>, std::string>>& keypointVector,
        const std::vector<std::vector<std::array<float,3>>>& candidates, const std::string& fileName,
        const bool humanReadable)
    {
        try
        {
            // Sanity check
            for (const auto& keypointPair : keypointVector)
                if (!keypointPair.first.empty() && keypointPair.first.getNumberDimensions() != 3
                    && keypointPair.first.getNumberDimensions() != 1)
                    error("keypointVector.getNumberDimensions() != 1 && != 3.", __LINE__, __FUNCTION__, __FILE__);
            // Record frame on desired path
            JsonOfstream jsonOfstream{fileName, humanReadable};
            jsonOfstream.objectOpen();
            // Add version
            // Version 0.1: Body keypoints (2-D)
            // Version 1.0: Added face and hands (2-D)
            // Version 1.1: Added candidates
            // Version 1.2: Added body, face, and hands (3-D)
            // Version 1.3: Added person ID (for temporal consistency)
            jsonOfstream.version("1.3");
            jsonOfstream.comma();
            // Add people keypoints
            addKeypointsToJson(jsonOfstream, keypointVector);
            // Add body part candidates
            if (!candidates.empty())
            {
                jsonOfstream.comma();
                addCandidatesToJson(jsonOfstream, candidates);
            }
            // Close object
            jsonOfstream.objectClose();
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void savePeopleTxt(const std::vector<std::pair<Array<float>, std::string>>& keypointVector)
    {
        try
        {
            // FILE STUFF
            std::vector<std::pair<float, float>> joint_locations;
            joint_locations.resize(keypointVector[1].first.getVolume()); //25 joint positions
            // std::map<float, float> joint_locations;

            std::ofstream myFile;
            myFile.open("/mnt/shared_drive/openpose/json/test.txt", std::ios::app);
            if (myFile.is_open()) {
                for (int i = 0; i < keypointVector[1].first.getVolume() - 1; i++) { 
                    myFile << keypointVector[1].first[i] << ", "; // x y confidence
                    if (i % 3 == 0) {
                        joint_locations[i] = std::make_pair(keypointVector[1].first[i], keypointVector[1].first[i+1]); //width, height pairs
                        std::cout << "Joints: " << joint_locations[i].first << ", " << joint_locations[i].second << std::endl;
                    }
                }
                myFile << keypointVector[1].first[keypointVector[1].first.getVolume() - 1] << std::endl;
                myFile.close();
            }

            // // DEPTH STUFF
            // rs2_error* e = 0;

            // // Create a context object. This object owns the handles to all connected realsense devices.
            // // The returned object should be released with rs2_delete_context(...)
            // rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
            // check_error(e);

            // /* Get a list of all the connected devices. */
            // // The returned object should be released with rs2_delete_device_list(...)
            // rs2_device_list* device_list = rs2_query_devices(ctx, &e);
            // check_error(e);

            // int dev_count = rs2_get_device_count(device_list, &e);
            // check_error(e);
            // printf("There are %d connected RealSense devices.\n", dev_count);
            // if (0 == dev_count)
            //     printf("no devices found");
            //     // return EXIT_FAILURE;

            // // Get the first connected device
            // // The returned object should be released with rs2_delete_device(...)
            // rs2_device* dev = rs2_create_device(device_list, 0, &e);
            // check_error(e);

            // print_device_info(dev);

            // // Create a pipeline to configure, start and stop camera streaming
            // // The returned object should be released with rs2_delete_pipeline(...)
            // rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
            // check_error(e);

            // // Create a config instance, used to specify hardware configuration
            // // The retunred object should be released with rs2_delete_config(...)
            // rs2_config* config = rs2_create_config(&e);
            // check_error(e);

            // // Request a specific configuration
            // rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
            // // rs2_config_disable_stream(config, RS2_STREAM_COLOR, &e);
            // check_error(e);

            // // Start the pipeline streaming
            // // The retunred object should be released with rs2_delete_pipeline_profile(...)
            // rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
            // if (e)
            // {
            //     printf("The connected device doesn't support depth streaming!\n");
            //     // exit(EXIT_FAILURE);
            // }

            // while (1)
            // {
            //     // This call waits until a new composite_frame is available
            //     // composite_frame holds a set of frames. It is used to prevent frame drops
            //     // The returned object should be released with rs2_release_frame(...)
            //     rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
            //     check_error(e);

            //     // Returns the number of frames embedded within the composite frame
            //     int num_of_frames = rs2_embedded_frames_count(frames, &e);
            //     check_error(e);

            //     int i;
            //     for (i = 0; i < num_of_frames; ++i)
            //     {
            //         // The retunred object should be released with rs2_release_frame(...)
            //         rs2_frame* frame = rs2_extract_frame(frames, i, &e);
            //         check_error(e);

            //         // Check if the given frame can be extended to depth frame interface
            //         // Accept only depth frames and skip other frames
            //         if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e))
            //             continue;

            //         // Get the depth frame's dimensions
            //         int width = rs2_get_frame_width(frame, &e);
            //         check_error(e);
            //         int height = rs2_get_frame_height(frame, &e);
            //         check_error(e);

            //         for (int j = 0; j < joint_locations.size(); ++j) {
            //             // Query the distance from the camera to the object in the center of the image
            //             // float dist_to_center = rs2_depth_frame_get_distance(frame, width / 2, height / 2, &e);
            //             float dist_to_center = rs2_depth_frame_get_distance(frame, joint_locations[j].first, joint_locations[j].second, &e);
            //             check_error(e);

            //             // Print the distance
            //             // printf("The camera is facing an object %.3f meters away at (%d, %d).\n", dist_to_center, joint_locations[j].first, joint_locations[j].second);
            //             std::ofstream myfile("output.txt", std::ios::app);
            //             if (myfile.is_open()){
            //                 myfile << "(" << joint_locations[j].first << "," << joint_locations[j].second << ") ,"<< dist_to_center << std::endl;
            //                 myfile.close();
            //             }
            //         }

            //         rs2_release_frame(frame);
            //     }

            //     rs2_release_frame(frames);
            // }

            // printf("HELLOOOO");
            // // Stop the pipeline streaming
            // rs2_pipeline_stop(pipeline, &e);
            // check_error(e);

            // // Release resources
            // rs2_delete_pipeline_profile(pipeline_profile);
            // rs2_delete_config(config);
            // // rs2_delete_pipeline(pipeline);
            // rs2_delete_device(dev);
            // rs2_delete_device_list(device_list);
            // rs2_delete_context(ctx);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error opening file." << std::endl;
        }
    }

    void saveImage(const Matrix& matrix, const std::string& fullFilePath,
                   const std::vector<int>& openCvCompressionParams)
    {
        try
        {
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(matrix);
            if (!cv::imwrite(fullFilePath, cvMat, openCvCompressionParams))
                error("Image could not be saved on " + fullFilePath + ".", __LINE__, __FUNCTION__, __FILE__);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    Matrix loadImage(const std::string& fullFilePath, const int openCvFlags)
    {
        try
        {
            cv::Mat cvMat = cv::imread(fullFilePath, openCvFlags);
            if (cvMat.empty())
                opLog("Empty image on path: " + fullFilePath + ".", Priority::Max, __LINE__, __FUNCTION__, __FILE__);
            return OP_CV2OPMAT(cvMat);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return Matrix();
        }
    }

    std::vector<std::array<Rectangle<float>, 2>> loadHandDetectorTxt(const std::string& txtFilePath)
    {
        try
        {
            std::vector<std::array<Rectangle<float>, 2>> handRectangles;

            std::string line;
            std::ifstream jsonFile{txtFilePath};
            if (jsonFile.is_open())
            {
                while (std::getline(jsonFile, line))
                {
                    const auto splittedStrings = splitString(line, " ");
                    std::vector<float> splittedInts;
                    for (auto splittedString : splittedStrings)
                        splittedInts.emplace_back(std::stof(splittedString));
                    if (splittedInts.size() != 4u)
                        error("splittedInts.size() != 4, but splittedInts.size() = "
                              + std::to_string(splittedInts.size()) + ".", __LINE__, __FUNCTION__, __FILE__);
                    const Rectangle<float> handRectangleZero;
                    const Rectangle<float> handRectangle{splittedInts[0], splittedInts[1], splittedInts[2],
                                                         splittedInts[3]};
                    if (getFileNameNoExtension(txtFilePath).back() == 'l')
                        handRectangles.emplace_back(std::array<Rectangle<float>, 2>{handRectangle, handRectangleZero});
                    else
                        handRectangles.emplace_back(std::array<Rectangle<float>, 2>{handRectangleZero, handRectangle});
                }
                jsonFile.close();
            }
            else
                error("Unable to open file " + txtFilePath + ".", __LINE__, __FUNCTION__, __FILE__);

            return handRectangles;
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return {};
        }
    }
}
