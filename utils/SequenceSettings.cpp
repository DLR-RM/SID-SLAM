//
// Created by font_al on 4/6/19.
//

#include "SequenceSettings.h"
//IDNav::SequenceSettings IDNav::seqSet{}; // seqSet is global throughout IDNav library.

void IDNav::SequenceSettings::initialize(const string& sequenceData_yamlFile_, const string& datasetFolder_){
    sequenceData_yamlFile =  sequenceData_yamlFile_;
    datasetFolder = datasetFolder_;
    cv::FileStorage fs_sequenceSettings =  openFileStorage(sequenceData_yamlFile);
    cv::FileNode n_seqSet;

    n_seqSet = fs_sequenceSettings["sequenceSettings"];
    if (not(n_seqSet.empty())) {
        sequenceName = (std::string) (n_seqSet["sequenceName"]);
        sequenceFolder = datasetFolder + "/" + sequenceName;
        camera_yamlFile = (std::string) n_seqSet["cameraYAML"];
    }

    n_seqSet = fs_sequenceSettings["RGBD_dataset"];
    if (not(n_seqSet.empty())) {
        cout << BOLDBLACK_COUT << "Operation mode: RGB-D " << RESET_COUT<<endl;
        operationMode = "rgbd";
        timestampsToSeconds = (dataType) (n_seqSet["timestampsToSeconds"]); // Timestamps conversion to seconds (s/timeUnit)
        rgbFolder = sequenceFolder + (std::string) (n_seqSet["rgbFolder"]);
        numRGB = (int) n_seqSet["numRGB"];
        RGB_timestamps_txt = sequenceFolder + "rgb.txt";

        depthFolder = sequenceFolder + (std::string) (n_seqSet["depthFolder"]);
        numDepth = (int) n_seqSet["numDepth"];
        depth_timestamps_txt = sequenceFolder + +"depth.txt";
        depthConstant = n_seqSet["depthConstant"]; // Depth image conversion (m/depthUnit)

        associationsTxt = sequenceFolder + (std::string) (n_seqSet["associationsTxt"]);
    }

    n_seqSet = fs_sequenceSettings["RGBD_TUM_dataset"];
    if (not(n_seqSet.empty())) {
        cout << BOLDBLACK_COUT << "Operation mode: RGB-D " << RESET_COUT<<endl;
        operationMode = "rgbd";
        timestampsToSeconds = 1.0; // Timestamps conversion to seconds (s/timeUnit)

        rgbFolder = sequenceFolder + "/" + "rgb";
        numRGB = (int) n_seqSet["numRGB"];
        RGB_timestamps_txt = sequenceFolder + "/" + "rgb.txt";

        depthFolder = sequenceFolder + "/" + "depth";
        numDepth = (int) n_seqSet["numDepth"];
        depth_timestamps_txt = sequenceFolder + "/" + "depth.txt";

        associationsTxt = sequenceFolder + "/"+ (std::string) (n_seqSet["associationsTxt"]);
        groundtruthTxt = sequenceFolder + "/"+ (std::string) (n_seqSet["groundtruthTxt"]);
    }

    // STEREO DATASET NOT IMPLEMENTED YET
    /*n_seqSet = fs_sequenceSettings["stereoDataset"];
    if (not(n_seqSet.empty())) {
        cout << "\nOperation mode: Stereo " << endl;
        operationMode = "stereo";

        left_rgbFolder = sequenceFolder + (std::string) (n_seqSet["leftFolder"]);
        right_rgbFolder = sequenceFolder + (std::string) (n_seqSet["rightFolder"]);
        RGB_hz = (dataType) (n_seqSet["RGB_hz"]);
        numRGB = (int) n_seqSet["numRGB"];
    }*/

#ifdef SEMANTIC_SEGMENTATION
    n_seqSet = fs_sequenceSettings["imageMask"];
    if (not(n_seqSet.empty())){
        cout << "Mask information: FOUND " << endl;
        thereAreImageMasks = true;
        maskFolder = sequenceFolder + (std::string) (n_seqSet["maskFolder"]);
        mask_hz = (int) n_seqSet["mask_hz"];
        numMask = (int) n_seqSet["numMask"];
    }
#endif

    fs_sequenceSettings.release();
}

void IDNav::SequenceSettings::loadImages(int headerTxtFilesSize){

    if(operationMode == "rgbd"){

        ifstream file(associationsTxt); // Check if there is association file available
        if(file.good()){
            cout <<"Associations file = " <<  associationsTxt << " : FOUND"<< endl;
            createFileListFromAssociationsFile(associationsTxt, 4,0);

            std::cout << "  num RGB found = " << RGB_list.size() <<" / "<< numRGB << std::endl;
            std::cout << "  num Depth found = " << depthList.size() <<" / "<< numDepth << std::endl;
            numRGB = RGB_list.size();
            numDepth = depthList.size();
        }else{
            cout <<BOLDBLACK_COUT<< "Association file: " << RED_COUT <<  associationsTxt << " :  NOT FOUND"<<  "\033[0m" <<endl;
            cout << "Read images from folders : "<< endl;
            cout << "  rgb : "<< rgbFolder << "\n  depth : "<< depthFolder <<  endl;
            createFileListFromFolder(rgbFolder, RGB_list);
            createFileListFromFolder(depthFolder, depthList);

            std::cout << "  num RGB found = " << RGB_list.size() <<" / "<< numRGB << std::endl;
            if(RGB_list.size() != numRGB) throw std::invalid_argument( "RGB not found" );
            std::cout << "  num Depth found = " << depthList.size() <<" / "<< numDepth << std::endl;
            if(depthList.size() != numDepth) throw std::invalid_argument( "depth not found" );

            ifstream fileRGB(RGB_timestamps_txt);
            ifstream fileDepth(depth_timestamps_txt);
            if(fileRGB.good()&&fileDepth.good()){
                cout << BOLDBLACK_COUT<< "Load timestamps with conversion " << timestampsToSeconds << " (s/time unit) from: "<< RESET_COUT<< endl;
                cout << "  " << depth_timestamps_txt << endl;
                cout << "  " << RGB_timestamps_txt << endl;

                vector<vector<string>> RGB_txt = read_txt(RGB_timestamps_txt,2,headerTxtFilesSize);
                readTimestampList(RGB_list_timestamp, RGB_txt, timestampsToSeconds);
                vector<vector<string>> depth_txt = read_txt(depth_timestamps_txt,2,headerTxtFilesSize);
                readTimestampList(depthList_timestamp, depth_txt, timestampsToSeconds);

                if(RGB_list_timestamp.size() != numRGB) throw std::invalid_argument( "RGB_list_timestamp not found" );
                if(depthList_timestamp.size() != numDepth) throw std::invalid_argument( "depthList_timestamp not found" );
            }
            else{
                cout << BOLDBLACK_COUT<< "Load timestamps: "<<RED_COUT<< "No rgb.txt and depth.txt with timestamps in "<< sequenceFolder << RESET_COUT<< endl;
                cout <<"  Generate timestamps with frequence: "<< RGB_hz << " (hz)" << RESET_COUT <<endl;

                createTimestampList(RGB_list, RGB_list_timestamp, RGB_hz);
                createTimestampList(depthList, depthList_timestamp,depth_hz);
            }
        }
    }

    if(operationMode == "stereo"){
        cout << "Loading left images from folder: "  << left_rgbFolder << endl;
        createFileListFromFolder(left_rgbFolder, left_RGB_list);
        createTimestampList(left_RGB_list, RGB_list_timestamp, RGB_hz);
        std::cout << "   num RGB left found = " << left_RGB_list.size() <<" / "<< numRGB << std::endl;
        if(left_RGB_list.size() != numRGB) throw std::invalid_argument( "RGB not found" );

        cout << "Loading right images from folder: "  << right_rgbFolder << endl;
        createFileListFromFolder(right_rgbFolder, right_RGB_list);
        std::cout << "   num RGB right found = " << left_RGB_list.size() <<" / "<< numRGB << std::endl;
        if(right_RGB_list.size() != numRGB) throw std::invalid_argument( "RGB not found" );
    }

    if(thereAreImageMasks){
        cout << "Loading mask images from folder: "  << maskFolder << endl;
        createFileListFromFolder(maskFolder, mask_list);
        std::cout << "   num masks found = " << mask_list.size() <<" / "<< numRGB << std::endl;
        if(mask_list.size() != numMask) throw std::invalid_argument( "RGB not found" );
    }

}

void IDNav::SequenceSettings::loadGroundtruth(int headerTxtFilesSize){
    if(operationMode == "rgbd"){
        ifstream fileGt(groundtruthTxt); // Check if there is association file available
        if(fileGt.good()) {
            cout << "Groundtruth file = " << groundtruthTxt << " : FOUND" << endl;
            vector<vector<string>> groundtruthFile = read_txt(groundtruthTxt, 8, 0);
            vec7 gt;
            size_t frameId{0};
            for (vector<string> &line: groundtruthFile) {
                gt(0) = stod(line[1]);
                gt(1) = stod(line[2]);
                gt(2) = stod(line[3]);
                gt(3) = stod(line[4]);
                gt(4) = stod(line[5]);
                gt(5) = stod(line[6]);
                gt(6) = stod(line[7]);
                groundtruth.insert({frameId,gt});
                ++frameId;
            }
            cout << "    # gt = " <<  groundtruth.size() << " : FOUND" << endl;
        }
    }
}

void IDNav::SequenceSettings::createFileListFromAssociationsFile(const string &filePath, const size_t& numCols, int titleSize) {
    vector<vector<string>> associationsFile = read_txt(filePath, numCols, titleSize);
    for (vector<string>& line: associationsFile){
        if(line[1].find("depth") != std::string::npos){
            depthList_timestamp.push_back(timestampsToSeconds*stod(line[0]));
            depthList.push_back(sequenceFolder + "/" + line[1]);
        }
        if(line[1].find("rgb") != std::string::npos){
            RGB_list_timestamp.push_back(timestampsToSeconds*stod(line[0]));
            RGB_list.push_back(sequenceFolder + "/" + line[1]);
        }
        if(line[3].find("depth") != std::string::npos){
            depthList_timestamp.push_back(timestampsToSeconds*stod(line[2]));
            depthList.push_back(sequenceFolder + "/" + line[3]);
        }
        if(line[3].find("rgb") != std::string::npos){
            RGB_list_timestamp.push_back(timestampsToSeconds*stod(line[2]));
            RGB_list.push_back(sequenceFolder + "/" + line[3]);
        }
    }
}

// Block of functions to write log files ///////////////////////////////////////////////////////////////////////////////

void IDNav::SequenceSettings::setLogFiles(const int& experimentName ,const int& numExperiments, string resultsFolder_){

    resultsFolder = resultsFolder_;
    // Padding zeros
    int numZeros = to_string(numExperiments*10).size();
    fileName = to_string(experimentName);
    string zeros = "0";
    for(int iZero{}; iZero < numZeros-fileName.size()-1; ++iZero){
        zeros += "0";
    }
    fileName = zeros + fileName;

    string path{};

    cameraTrajectory_TUMformat_Log = new ofstream();
    path = resultsFolder  +"/"+ fileName + "_CameraTrajectory.txt";
    cameraTrajectory_TUMformat_Log->open(path, std::ios::trunc | std::ios::out);
    cameraTrajectory_TUMformat_Log->precision(12);

    keyframeTrajectory_TUMformat_Log = new ofstream();
    path = resultsFolder  +"/"+ fileName + "_KeyFrameTrajectory.txt";
    keyframeTrajectory_TUMformat_Log->open(path, std::ios::trunc | std::ios::out);
    keyframeTrajectory_TUMformat_Log->precision(12);

    finalReport_Log = new ofstream();
    path = resultsFolder  +"/"+ fileName + "_finalReport_log.txt";
    finalReport_Log->open(path, std::ios::trunc | std::ios::out);
    finalReport_Log->precision(12);

    information_Log = new ofstream();
    path = resultsFolder  +"/"+ fileName + "_information_Log.txt";
    information_Log->open(path, std::ios::trunc | std::ios::out);
    information_Log->precision(12);

    covCorr_Log = new ofstream();
    path = resultsFolder  +"/"+ fileName + "_covCorr_Log.txt";
    //path = "/home/afontan/toDelete/"+ fileName + "_covCorr_Log.txt";
    covCorr_Log->open(path, std::ios::trunc | std::ios::out);
    covCorr_Log->precision(12);

    ifstream f(path);
    if (f.good())  cout << BOLDBLACK_COUT<< "Created log files: "<<RESET_COUT<< endl;
    else           cout << BOLDBLACK_COUT<< "Create log files: " << BOLDRED_COUT << " FAILED"<< RESET_COUT << endl;
    cout << "  log file : " << resultsFolder + "/" + fileName + "_finalTraj_TUMformat_log.txt" << endl;
    cout << "  log file : " << resultsFolder + "/" + fileName + "_finalReport_log.txt" << endl;
}

void IDNav::SequenceSettings::closeLogFiles(){

    cameraTrajectory_TUMformat_Log->close();
    delete cameraTrajectory_TUMformat_Log;

    keyframeTrajectory_TUMformat_Log->close();
    delete keyframeTrajectory_TUMformat_Log;

    finalReport_Log->close();
    delete finalReport_Log;

    information_Log->close();
    delete information_Log;

    covCorr_Log->close();
    delete covCorr_Log;
}

void IDNav::SequenceSettings::removeFilesFromDirectory(const string& directoryPath, const string& extensionToRemove){
    DIR *dpath = opendir(directoryPath.c_str());
    string fileToRemove{};
    while (struct dirent *dentry = readdir(dpath)) {
        if ((dentry->d_name[0] != '.') && (to_string(dentry->d_name[0]) != "..") &&
            (string(dentry->d_name).find(extensionToRemove) != std::string::npos)) {
            fileToRemove = directoryPath + "/" + dentry->d_name;
            remove(fileToRemove.c_str());
        }
    }
}