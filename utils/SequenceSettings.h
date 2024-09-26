//
// Created by font_al on 4/6/19.
//

#ifndef IDNAV_SEQUENCESETTINGS_H
#define IDNAV_SEQUENCESETTINGS_H

#include "file_functions.h"
namespace IDNav {

    // seqSet is global throughout IDNav library.
    class SequenceSettings {
    public:

        string sequenceData_yamlFile{};

        string operationMode{};
        string sequenceName{},sequenceFolder{};
        string datasetFolder{};
        string rgbFolder{}, depthFolder{}, left_rgbFolder{}, right_rgbFolder{}, maskFolder{};
        size_t numRGB{1000}, numDepth{1000}, numMask{1000};
        string resultsFolder{}, fileName{};

        string associationsTxt{};
        string groundtruthTxt{};
        string RGB_timestamps_txt{}, depth_timestamps_txt{};
        dataType timestampsToSeconds{1.0}; // In case the timestamps provided in .txt are not in seconds;

        dataType mask_hz{};
        dataType RGB_hz{},depth_hz{};
        dataType RGB_seconds{};

        string camera_yamlFile{};
        float depthConstant{5000.0};

        vector<string> RGB_list{};
        vector<dataType> RGB_list_timestamp{};
        vector<string> depthList{};
        vector<dataType> depthList_timestamp{};

        vector<string> left_RGB_list{};
        vector<string> right_RGB_list{};

        vector<string> mask_list{};

        bool thereAreImageMasks{false};
        std::ofstream* cameraTrajectory_TUMformat_Log{};
        std::ofstream* keyframeTrajectory_TUMformat_Log{};
        std::ofstream* finalReport_Log{};
        std::ofstream* information_Log{};
        std::ofstream* covCorr_Log{};

        map<size_t,vec7> groundtruth{};

        SequenceSettings() = default;
        void initialize(const string& pathDataset, const string& datasetFolder_);
        void loadImages(int headerTxtFilesSize = 0);
        void loadGroundtruth(int headerTxtFilesSize = 0);
        void createFileListFromAssociationsFile(const string &filePath, const size_t& numCols, int titleSize = 0);

        // Block of functions to write log files.
        static void removeFilesFromDirectory(const string& directoryPath, const string& extensionToRemove);
        void setLogFiles(const int& experimentName ,const int& numExperiments, string resultsFolder_);
        void closeLogFiles();

    };

    //extern SequenceSettings seqSet;
}
#endif //IDNAV_SEQUENCESETTINGS_H
