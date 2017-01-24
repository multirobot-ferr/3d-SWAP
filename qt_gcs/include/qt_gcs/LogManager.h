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

#include <chrono>
#include <fstream>
#include <mutex>
#include <unordered_map>
class LogManager{
public:
    static void init();
    static LogManager * get();
    static void end();

    void error(std::string _tag, std::string _message);
    void warning(std::string _tag, std::string _message);
    void status(std::string _tag, std::string _message);

private:
    LogManager();
    ~LogManager();

    void write(std::string _tag, std::string _type, std::string _msg);
    void checkTag(std::string _tag);

private:
    std::string mBasePath;
    static LogManager *mInstance;

    std::chrono::high_resolution_clock::time_point mInitPoint;
    std::unordered_map<std::string, std::ofstream*> mLogFiles;
    std::unordered_map<std::string, std::mutex> mSecureWrite;
};
