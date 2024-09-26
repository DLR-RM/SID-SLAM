//
// Created by afontan on 27/7/20.
//

#ifndef IDNAV_FILE_FUNCTIONS_H
#define IDNAV_FILE_FUNCTIONS_H

#include "test_functions.h"

namespace IDNav {

    cv::FileStorage openFileStorage(const string& pathYamlFile);

    // Block of functions to load images from a folder
    void createFileListFromFolder(const string &listPath, vector<string> &fileList);
    void createTimestampList(vector<string> &_imageList, vector<dataType> &_imageList_timestamp, const dataType& freq_hz);
    bool isImage(const string &fname_);

    // Block of functions to load images and timestamps from a  .txt file
    vector<vector<string>> read_txt(const string &filePath, const size_t& numCols, int titleSize = 0);
    void readTimestampList(vector<dataType> &_imageList_timestamp, const vector<vector<string>>& txtFile, const dataType& timeUnitToSeconds);
    void display_txt(const vector<vector<string>>& txtFile);
    template <typename T> T convert_to (const string &str){
        istringstream ss(str);
        T num;
        ss >> num;
        return num;
    }
}

#endif //IDNAV_FILE_FUNCTIONS_H
