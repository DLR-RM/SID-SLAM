//
// Created by font_al on 6/11/21.
//

#ifndef IDNAV_PROFILER_H
#define IDNAV_PROFILER_H

#include <utility>
namespace IDNav {

    // This class provides functions to track the performance of a process by simply placing the 'begin' and 'end'
    // functions appropriately.
    class Profiler {

    private:
        std::string name{};
        std::string whiteSpaces{};
        std::string module{};
        std::string function{};
        std::string color{BOLDMAGENTA_COUT};
        std::chrono::high_resolution_clock::time_point t1{},t2{};
        std::chrono::duration<float, std::milli> timeLastExecution{0};
        float totalTime{0}, meanTime{0}, maxTime{0.0}, minTime{numeric_limits<float>::max()};
        size_t numberOfExecutions{0};

    public:

        explicit Profiler(std::string  functionName, std::string whiteSpaces = "" ,std::string module = "Profiler", std::string function  = " ", std::string color = BOLDMAGENTA_COUT) :
                name{std::move(functionName)},module{module},whiteSpaces{whiteSpaces},function(function),color(color){};
        explicit Profiler() = default;

        void begin() {
            t1 = std::chrono::high_resolution_clock::now();
            ++numberOfExecutions;
#ifdef COUT_PIPELINE_TIMING
            cout << color << whiteSpaces << "["<< module + "] " + function + "() : Begin " + name << RESET_COUT << endl;
#endif
        }

        void end() {
            t2 = std::chrono::high_resolution_clock::now();
            timeLastExecution = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
            if(timeLastExecution.count() > 0){
                totalTime += timeLastExecution.count();
                if(timeLastExecution.count() > maxTime) maxTime = timeLastExecution.count();
                if(timeLastExecution.count() < minTime) minTime = timeLastExecution.count();
            }
#ifdef COUT_PIPELINE_TIMING
            cout << color << whiteSpaces << "["<< module + "] " + function + "() : Finish " + name << RESET_COUT << " , execution time = " << timeLastExecution.count() << " ms" << endl;
#endif
        }

        dataType getTime_secs(){
            t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float, std::milli> timeLastExecution_ = std::chrono::duration_cast<std::chrono::duration<float>>(t2 - t1);
            return 0.001*timeLastExecution_.count();
        }

        float getMeanTime(){
            meanTime = totalTime / (float)numberOfExecutions;
            return meanTime;
        }

        void computeProfile(){
            getMeanTime();
        }

        float getLastExecutionTime(){
            return timeLastExecution.count();
        }

        void showProfile() {
            computeProfile();
            cout << BOLDMAGENTA_COUT << "["<< module + "] " + function + "() : Profiling :  execution time = " << timeLastExecution.count() << " ms" << endl;

            std::cout <<BOLDBLACK_COUT << "Profiling of function = " << RESET_COUT << function << std::endl;
            std::cout << "  Total Time              = " << totalTime << " ms" << std::endl;
            std::cout << "  Number of Executions    = " << numberOfExecutions << std::endl;
            std::cout << "  Mean Time               = " << meanTime << " ms" << std::endl;
            std::cout << "  Min Time                = " << minTime << " ms" << std::endl;
            std::cout << "  Max Time                = " << maxTime << " ms\n" << std::endl;
        }
    };
}


#endif //IDNAV_PROFILER_H