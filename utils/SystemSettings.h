
#ifndef IDNAV_SETTINGS_H
#define IDNAV_SETTINGS_H

#include "file_functions.h"

namespace IDNav{

    // sysSet is global throughout IDNav library.
    class SystemSettings {
    public:

        //pointSelector_hgp (pointSelector.h)
        int numPointsToSelectWithInformation{64};
        int numPointsToExtract{256};
        float minPhotoGradient{15.0f};

        //Is keyframe (trackingFrontend.h)
        dataType maxInformationLoss{5.0};

        // Visibility limits
        double maxLambdaVisParam{1.2};
        double maxKeyframeDistance{1.0};
        double minVisiblePointsRatio{0.2};

        // Tracking
        double tstudent2Probability{95.0};
        float minInliersRatio{0.70};

        // Window optimization
        int numKeyframesWindowOptimization{10};
        bool optimizeDephts{true};
        double tstudent2ProbabilityWindow{95.0};

        // Features
        dataType minResponse{0.00001};
        size_t numOctaves{3};
        size_t numSublevels{1};
        size_t numThreads{1};
        dataType matchingThreshold{1.0};
        dataType matchingDistance{0.9};
        double featureBias{0.0};

        // Visualizer
        bool visualization{true}; // Enable pangolin visualization
        bool saveSequence{false}; // Save 'pangolin map' and 'tracked frame' as .png

        SystemSettings() = default;
        void initialize(const string& path_systemSettings_yamlFile);
    };

}


#endif //IDNAV_SETTINGS_H
