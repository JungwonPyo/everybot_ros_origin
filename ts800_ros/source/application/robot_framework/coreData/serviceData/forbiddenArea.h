#pragma once
#include "commonStruct.h"

typedef struct _tForbiddenArea
{
    _tForbiddenArea() {}

    short type;
    std::list<tForbiddenLine> line;
    short lineNumber;
    std::list<tForbiddenRect> rect;
    short rectNumber;
}tForbiddenArea;

class CForbiddenArea
{
private:
    tForbiddenArea forbiddenArea;
public:
    CForbiddenArea();
    ~CForbiddenArea();

    void setForbiddenLine();
    void setForbiddenRect();
    std::list<tForbiddenLine> getForbiddenLine();
    std::list<tForbiddenRect> getForbiddenRect();

};