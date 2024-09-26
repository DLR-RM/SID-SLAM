//
// Created by afontan on 27/7/20.
//

#include "file_functions.h"

cv::FileStorage IDNav::openFileStorage(const string& pathYamlFile){
    cv::FileStorage tempFile(pathYamlFile, cv::FileStorage::READ);
    if (!tempFile.isOpened()) {
        std::cerr << "Failed to open: " << pathYamlFile << std::endl;
    }
    else{
        std::cout << BOLDBLACK_COUT << pathYamlFile << RESET_COUT <<" : " << GREEN_COUT << "FOUND" << RESET_COUT<<std::endl;
    }
    return tempFile;
}

// Block of functions to load images from a folder /////////////////////////////////////////////////////////////////////

// This function creates a list of paths for the images contained on a given folder
void IDNav::createFileListFromFolder(const string &listPath, vector<string> &fileList) {
    fileList.clear();
    bool wildcard = listPath.find("*") != listPath.find("?");
    string listBase = listPath;
    string listMatch = "";
    if (wildcard) {
        size_t pos = listPath.find_last_of("/");
        if (pos != std::string::npos) {
            listBase = listPath.substr(0, pos);
            listMatch = listPath.substr(pos + 1);
        } else {
            listBase = ".";
            listMatch = listPath;
        }
    }
    struct stat filestat;
    if (stat(listBase.c_str(), &filestat)) {
    } else if (S_ISDIR(filestat.st_mode)) {
        DIR *dpath = opendir(listBase.c_str());
        if (!dpath) {
        } else {
            while (struct dirent *dentry = readdir(dpath)) {
                if ((dentry->d_name[0] != '.')&& (dentry->d_name[0] != '..')){
                    string imgPath = listBase + '/' + dentry->d_name;
                    if (isImage(imgPath)) {
                        fileList.push_back(listBase + '/' + dentry->d_name);
                        //cout << fileList.back() << endl;
                    }
                }
            }
        }
        closedir(dpath);
    } else {
        std::string line;
        std::ifstream f(listBase.c_str());
    }
    std::sort(fileList.begin(), fileList.end());
    /*for (int i{}; i < fileList.size(); ++i ){
        cout <<fileList[i] << endl;
    }
    wf(".");*/
}

// This function creates a list of timestamps for a given list of images
void IDNav::createTimestampList(vector<string> &_imageList, vector<dataType> &_imageList_timestamp, const dataType& freq_hz) {
    dataType periodSeconds = 1 / freq_hz;
    for (int idImage{0}; idImage < _imageList.size(); ++ idImage) {
        _imageList_timestamp.push_back(periodSeconds * idImage); // Generating timestamp from given frequency
    }
}

// This function checks if a file is an image.
bool IDNav::isImage(const string &fname_) {
    string fext = fname_.substr(fname_.find_last_of(".") + 1);
    return !(strcasecmp(fext.c_str(), "JPG") &&
             strcasecmp(fext.c_str(), "JPEG") &&
             strcasecmp(fext.c_str(), "EXR") &&
             strcasecmp(fext.c_str(), "PNG"));
}

// Block of functions to load images and timestamps from a  .txt file //////////////////////////////////////////////////
// This function reads a txt file and converts it into a vector of strings
vector<vector<string>> IDNav::read_txt(const string &filePath, const size_t& numCols, int titleSize){
    vector<vector<string>> txtFile{};
    ifstream file(filePath);
    string line, dataUnit_string;

    if(file.good()){
        file.clear();
        file.seekg(0, std::ios::beg);
        for (int j = 0; j < titleSize; ++j) {
            getline(file, line);
        }
        while (getline(file, line)){
            stringstream line_stream(line);
            vector<string> row_numeric{};
            for (int j = 0; j < numCols; ++j) {
                getline(line_stream, dataUnit_string, ' ');
                row_numeric.push_back(dataUnit_string);
            }
            txtFile.push_back(row_numeric);
        }
    }
    else{
        throw std::invalid_argument( "File not found : " + filePath);
    }
    return txtFile;
}

// This function creates a list of paths and a list of timestamps for the images from a vector of strings
void IDNav::readTimestampList(vector<dataType> &_imageList_timestamp,
        const vector<vector<string>>& txtFile, const dataType& timestampsToSeconds){
    _imageList_timestamp.clear();
    size_t numRows = txtFile.size();
    for(size_t rowList{}; rowList < numRows; ++rowList){
        _imageList_timestamp.push_back(convert_to<dataType>(txtFile[rowList][0])*timestampsToSeconds);
    }
}

// This function displays a vector of strings
void IDNav::display_txt(const vector<vector<string>>& txtFile){
    int numRows = txtFile.size();
    int numCols = txtFile[0].size();
    for(size_t row{0}; row < numRows; ++row){
        for(size_t col{0}; col < numCols; ++col){
            cout << txtFile[row][col] <<" ";
        }
        cout << "\n";
    }
    cout << "\nnumRows = " << numRows << endl;
    cout << "numCols = " << numCols << endl;
    cout << "numElements = " << numRows*numCols << endl;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
