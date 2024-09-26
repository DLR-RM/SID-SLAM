
#include "../../include/Pt.h"

void IDNav::IdNavPoint::setProjectionPointers(){
    u.reset(new dataType[ptSize]);
    v.reset(new dataType[ptSize]);

    xn.reset(new dataType[ptSize]);
    yn.reset(new dataType[ptSize]);
    lambda.reset(new dataType[ptSize]);

    X.reset(new dataType[ptSize]);
    Y.reset(new dataType[ptSize]);
    Z.reset(new dataType[ptSize]);

    xnRef.reset(new dataType[ptSize]);
    ynRef.reset(new dataType[ptSize]);
}

void IDNav::Pt::setProjectionPointers(){
    IDNav::IdNavPoint::setProjectionPointers();

    I.reset(new dataType[ptSize]);
    for (int iMask{}; iMask < ptSize; ++iMask) I[iMask] = I_ref[iMask][0];
    Gu.reset(new dataType[ptSize]);
    Gv.reset(new dataType[ptSize]);
}
