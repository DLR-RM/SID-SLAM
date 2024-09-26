//
// Created by alejandro on 5/24/20.
//

#ifndef ID_INFORMATIONFUNCTIONS_H
#define ID_INFORMATIONFUNCTIONS_H

#include "../utils/definesAndIncludes.h"

namespace IDNav{
    dataType computeDiffEntropy(const dataType& infMatrixDet,const dataType& infConstant);
    dataType computeDiffEntropy_pose(const matX& poseHessian);
    matX computeDiffEntropy_pose_DoF(const matX& poseHessian);

    void generateBlockSequence(int numBlocks ,std::vector<int>& blockOrder);
    void reorderMatrix(matX& H, std::vector<int> blockOrder, int numBlocks, int sizeBlock);
}


#endif //ID_INFORMATIONFUNCTIONS_H
