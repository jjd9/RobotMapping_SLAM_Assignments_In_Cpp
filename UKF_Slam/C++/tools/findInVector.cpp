#include "tools.h"

// take from https://thispointer.com/c-how-to-find-an-element-in-vector-and-get-its-index/
std::pair<bool, int > findInVector(const std::vector<int>  & vecOfElements, int  & element)
{
    std::pair<bool, int > result;
    // Find given element in vector
    auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);
    if (it != vecOfElements.end())
    {
        result.second = distance(vecOfElements.begin(), it);
        result.first = true;
    }
    else
    {
        result.first = false;
        result.second = -1;
    }
    return result;
}