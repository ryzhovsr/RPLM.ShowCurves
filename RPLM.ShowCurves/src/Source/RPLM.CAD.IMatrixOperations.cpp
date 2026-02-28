#include "RPLM.CAD.EigenMatrixOperations.h"

IMatrixOperationsPtr IMatrixOperations::GetMatrixOperationsClass(OperationClass iClassName)
{
    switch (iClassName)
    {
    case eigen:
        return std::make_shared<EigenMatrixOperations>();
    default:
        return nullptr;
    }
}
