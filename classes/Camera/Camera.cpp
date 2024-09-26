//
// Created by font_al on 10/26/18.
//

#include "../../include/Camera.h"

// This function chooses a patch structure from a predefined set
void IDNav::Camera::selectPatch(size_t patchIdentifier){
    if (patchIdentifier > PATCH_SIZE) patchIdentifier = PATCH_SIZE;
    if (patchIdentifier <= 0) patchIdentifier = 1;

    switch (patchIdentifier){
        case 1:{
            patch_u[0] = 0; patch_v[0] = 0;
            patchSurface = 1;
            break;}

        case 5: {
            patch_u[0] = 0;  patch_v[0] = 0;  patch_u[1] = +1; patch_v[1] = -1; patch_u[2] = 0; patch_v[2] = -2;
            patch_u[3] = -1; patch_v[3] = +1; patch_u[4] =  0; patch_v[4] = +2;
            patchSurface = 2;
            break;}

        case 7:{
            patch_u[0] = 0; patch_v[0] = 0;  patch_u[1] = -1;patch_v[1] = -1; patch_u[2] = +1;patch_v[2] = -1;
            patch_u[3] = +1;patch_v[3] = +1; patch_u[4] = -1;patch_v[4] = +1; patch_u[5] = -2 ;patch_v[5] = 0;
            patch_u[6] = +2 ;patch_v[6] = 0;
            patchSurface = 3;
            break;}

        case 8:{
            patch_u[0] = 0; patch_v[0] = 0;  patch_u[1] = -1;patch_v[1] = -1; patch_u[2] = +1;patch_v[2] = -1;
            patch_u[3] = -1;patch_v[3] = +1; patch_u[4] = 0 ;patch_v[4] = -2; patch_u[5] = +2 ;patch_v[5] = 0;
            patch_u[6] = 0 ;patch_v[6] = +2; patch_u[7] = -2 ;patch_v[7] = 0;
            patchSurface = 3;
            break;
        }

        case 9:{
            patch_u[0] = 0; patch_v[0] = 0;  patch_u[1] = -1;patch_v[1] = -1; patch_u[2] = +1;patch_v[2] = -1;
            patch_u[3] = +1;patch_v[3] = +1; patch_u[4] = -1;patch_v[4] = +1; patch_u[5] = 0 ;patch_v[5] = -2;
            patch_u[6] = +2 ;patch_v[6] = 0; patch_u[7] = 0 ;patch_v[7] = +2; patch_u[8] = -2 ;patch_v[8] = 0;

            patchWeight[0] = 1;
            patchWeight[1] = 1/sqrt(3.0); patchWeight[2] = 1/sqrt(3.0);      //   5
            patchWeight[3] = 1/sqrt(3.0); patchWeight[4] = 1/sqrt(3.0);      //  1 2
            patchWeight[5] = 0.5; patchWeight[6] = 0.5;                            // 8 0 6
            patchWeight[7] = 0.5; patchWeight[8] = 0.5;                            //  4 3
            patchSurface = 3;                                                      //   7
            break;
        }

        default:{
            patch_u[0] = 0; patch_v[0] = 0;
            patchSurface = 1;
            break;
        }
    }

    // NOT TESTED. In case you want to scale the patches (Because of visualization purposes ???)
    /*double patchScale{5.0};
    cout << "PATCHES SCALED = "  << patchScale << endl;
    for (int i{0}; i < PATCH_SIZE; ++i){
               patch_u[i] = patchScale*patch_u[i];
               patch_v[i] = patchScale*patch_v[i];
               //patchSurface *= patchScale;
    }*/
}

// Grid functions

// This function adapts the grid to the input image
void IDNav::ImageGrid::build(const int& w_, const int& h_){
    w = w_;
    h = h_;
    sizeCellPixels_u = float(w)/float(numCells_u);
    sizeCellPixels_v = float(h)/float(numCells_v);

    u_max = w - imageMargin;
    v_max = h - imageMargin;

    std::shared_ptr<GridElement> gridElement{new GridElement()};
    grid.clear();
    for(int u_block{}; u_block < numCells_u; ++u_block){
        for(int v_block{}; v_block < numCells_v; ++v_block) {

            gridElement.reset(new GridElement());

            gridElement->ui = int(roundf(float(u_block) * sizeCellPixels_u));
            gridElement->vi = int(roundf(float(v_block) * sizeCellPixels_v));
            gridElement->uf = int(roundf(float(u_block + 1) * sizeCellPixels_u - 1));
            gridElement->vf = int(roundf(float(v_block + 1) * sizeCellPixels_v - 1));

            if (gridElement->ui < u_min) gridElement->ui = u_min;
            if (gridElement->vi < v_min) gridElement->vi = v_min;
            if (gridElement->uf >= u_max) gridElement->uf = u_max;
            if (gridElement->vf >= v_max) gridElement->vf = v_max;

            grid.push_back(gridElement);
        }
    }
}

void IDNav::ImageGrid::get_cell(int& uCell, int& vCell, const int& u, const int& v){
    uCell = u/sizeCellPixels_u;
    vCell = v/sizeCellPixels_v;
}
