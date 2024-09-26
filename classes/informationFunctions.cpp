//
// Created by alejandro on 5/24/20.
//

#include "../utils/informationFunctions.h"

namespace IDNav{
    dataType computeDiffEntropy(const dataType& infMatrixDet,const dataType& infConstant){
        return -(infConstant - 0.5*log2(infMatrixDet));
    }

    dataType computeDiffEntropy_pose(const matX& poseHessian){
        return computeDiffEntropy(poseHessian.determinant(),12.2825735);
    }

    matX computeDiffEntropy_pose_DoF(const matX& poseHessian){

        matX entropy = matX::Zero(1, 6);
        matX COV_aux_ = poseHessian.inverse();
        matX COV_aux;
        matX schurComplement;
        vector<int> blockOrder;
        for (size_t iDOF{0}; iDOF < 6; ++iDOF) {
            blockOrder.clear();
            blockOrder.push_back(iDOF);
            generateBlockSequence(6, blockOrder);
            COV_aux = COV_aux_;
            reorderMatrix(COV_aux, blockOrder, 6, 1);
            schurComplement = COV_aux.block<1, 1>(0, 0) -
                              COV_aux.block<1, 5>(0, 1) * COV_aux.block<5, 5>(1, 1).inverse() *
                              COV_aux.block<5, 1>(1, 0);
            entropy(0, iDOF) = computeDiffEntropy(1/schurComplement.determinant(),2.0470955);
        }
        return entropy;
    }

    void reorderMatrix(matX& H, std::vector<int> blockOrder, int numBlocks, int sizeBlock){
        std::vector<int>& blockOrder_ = blockOrder;
        generateBlockSequence(numBlocks,blockOrder_);

        matX Htemp = H;
        int i, j;
        for (int iKeyframe{}; iKeyframe < numBlocks; ++iKeyframe) {
            for (int jKeyframe{}; jKeyframe < numBlocks; ++jKeyframe) {

                i = blockOrder[iKeyframe];
                j = blockOrder[jKeyframe];

                Htemp.block(sizeBlock * iKeyframe, sizeBlock * jKeyframe, sizeBlock, sizeBlock) =
                        H.block(sizeBlock * i, sizeBlock * j,sizeBlock, sizeBlock);
            }
        }
        H = Htemp;
    }

    void generateBlockSequence(int numBlocks ,std::vector<int>& blockOrder) {
        std::vector<int>& keyframesOrder_ = blockOrder;
        bool addIndex{};
        for (int i{}; i < numBlocks; ++i) {
            addIndex = true;
            for (int j{}; j < keyframesOrder_.size(); ++j) {
                if (i == keyframesOrder_[j]) {
                    addIndex = false;
                    break;
                }
            }
            if(addIndex){blockOrder.push_back(i);}
        }
    }
}


