//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 Pablo Ramon Soria
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <qt_gcs/LogManager.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <sys/stat.h>

LogManager* LogManager::mInstance = nullptr;

//----------------------------------------------------------------------------------------------------------------------
void LogManager::init(){
    if(mInstance == nullptr){
        mInstance = new LogManager();
    }
}

//----------------------------------------------------------------------------------------------------------------------
LogManager * LogManager::get(){
    return mInstance;
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::end() {
    delete mInstance;
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::error(std::string _tag, std::string _message) {
    checkTag(_tag);
    write(_tag, "ERROR", _message);
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::warning(std::string _tag, std::string _message) {
    checkTag(_tag);
    write(_tag, "WARNING", _message);
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::status(std::string _tag, std::string _message) {
    checkTag(_tag);
    write(_tag, "", _message);
}

//----------------------------------------------------------------------------------------------------------------------
LogManager::LogManager() {
    std::time_t now_c = std::chrono::high_resolution_clock::to_time_t(std::chrono::high_resolution_clock::now() - std::chrono::hours(24));
    std::stringstream ss;
    mBasePath = std::string(getenv("HOME"))+"/.mbzirc_gcs";
    mkdir(mBasePath.c_str(), 0777);
    ss << mBasePath << "/log_" << std::put_time(std::localtime(&now_c), "%F_%T");
    // create folder for current run
    mkdir(ss.str().c_str(),0777);
    // init files


    std::cout << "Created log files in: " << ss.str() << std::endl;
    mInitPoint = std::chrono::high_resolution_clock::now();
}

//----------------------------------------------------------------------------------------------------------------------
LogManager::~LogManager() {
    for(auto &it:mLogFiles){
        it.second->close();
    }
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::write(std::string _tag, std::string _type, std::string _msg) {
    double time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - mInitPoint).count();
    std::string logLine = "["+std::to_string(time/1000)+"] ["+_type+"] "+_msg + "\n";
    mSecureWrite[_tag].lock();
    (*mLogFiles[_tag]) << logLine;
    (*mLogFiles[_tag]).flush();
    mSecureWrite[_tag].unlock();
}

//----------------------------------------------------------------------------------------------------------------------
void LogManager::checkTag(std::string _tag) {
    for(auto iter = mLogFiles.begin(); iter != mLogFiles.end(); iter++){
        if(iter->first == _tag){
            return;
        }
    }
    mLogFiles[_tag] = new std::ofstream("log_"+_tag+".txt");
    mSecureWrite[_tag];
}
