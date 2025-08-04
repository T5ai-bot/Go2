#pragma once
#include <mutex>
#include <champ/motion_matching/MMOutput.h>
namespace champ::motion_matching {
class MotionMatchingModule {
public:
    static MotionMatchingModule& instance() {
        static MotionMatchingModule inst; return inst; }
    void setLatestOutput(const MMOutput& out){
        std::lock_guard<std::mutex> lk(mtx_); latest_ = out; }
    static MMOutput getLatestOutput() { return instance().get(); }
private:
    MMOutput get(){ std::lock_guard<std::mutex> lk(mtx_); return latest_; }
    std::mutex mtx_;  MMOutput latest_;
};
}
