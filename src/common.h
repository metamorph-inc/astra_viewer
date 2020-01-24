#pragma once

namespace astra_viewer {

    #define PUBLIC public:
    #define PRIVATE private:
    #define PROTECTED protected:
    #define C_PTR(type) PUBLIC typedef type* Ptr;
    #define SHARED_PTR(type) PUBLIC typedef std::shared_ptr<type> Shared_Ptr;
    #define UNIQUE_PTR(type) PUBLIC typedef std::unique_ptr<type> Unique_Ptr;

}