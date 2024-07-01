#include "path_controller/pathController.h"

CPathController::CPathController()
{
    setState(E_PATH_CTR_STATE::DOING);
}

CPathController::~CPathController() {}

CPathController::E_PATH_CTR_STATE CPathController::getState()
{
    return state;
}

void CPathController::setState(E_PATH_CTR_STATE state)
{
    this->state = state;
}