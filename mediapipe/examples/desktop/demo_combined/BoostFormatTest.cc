#include <boost/format.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    // 准备格式
    // boost::format fmt("Training Time: %s      Left Flexion: %s      Right Flexion: %s");
    // // std::cout << fmt.str() << std::endl;  // 没有%输入会报错
    // // 准备数据
    // std::string na_str("N/A");
    // double leftFlextion = 13.123456;
    // boost::format subfmt("%.1f°");
    // boost::format time_fmt("%02d:%02d");
    

    // // 格式化
    // subfmt % leftFlextion;
    // fmt % na_str % subfmt.str() % na_str;

    // // 获取最终需要的字符串
    // std::cout << fmt.str() << std::endl;


    // double rightFlextion = 29.00;
    // subfmt % rightFlextion;
    // time_fmt % 0 % 2;
    // fmt % time_fmt.str() % na_str % subfmt.str();
    // std::cout << fmt.str() << std::endl;


    std::chrono::_V2::system_clock::time_point start;
    start = std::chrono::system_clock::now();

    std::this_thread::sleep_for(std::chrono::milliseconds(1234));

    const auto end = std::chrono::system_clock::now();

    const auto ms_dur = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Duration in milliseconds: " << ms_dur << std::endl;


    return 0;
}








