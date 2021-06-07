#pragma once

#include <iostream>
#include <fstream>
#include <chrono>

class SuhanBenchmark
{
public:
    SuhanBenchmark() : beg_(hd_clock::now()) {}
    void reset() { beg_ = hd_clock::now(); }
    double elapsed() const { 
        return std::chrono::duration_cast<second>
            (hd_clock::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock hd_clock;
    typedef std::chrono::duration<double, std::ratio<1> > second;
    std::chrono::time_point<hd_clock> beg_;
};