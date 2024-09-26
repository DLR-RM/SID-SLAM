//
// Created by giub_ri 5 Nov 2021
//

#ifndef LCD_SYMBOL
#define LCD_SYMBOL

#include <iostream>
#include "limits.h"

typedef uint64_t u64;

namespace LCD {
    class Symbol {
    public:
        /**
         * Constructor for class Symbol.
         * @param robot_id          char array for robot id
         * @param keyframe_id       numeric value for unique keyframe id, max val: xxx
         * @param session_id        numeric value for session id, can be left empty
         */
        Symbol(const std::string& robot_id, const u64& keyframe_id, const u64& session_id = 0);

        /**
         * Constructor for class Symbol
         * @param data              packed symbol in 64 bit integer
         */
        Symbol(const u64& data);

        /**
         * getNumeric: returns the numeric value as a 64bit integer packing robot, keyframe and session ids.
         */
        u64 getNumeric() const;

        /**
         * getRobotId: returns a max 2 characters string denoting the robot id
         */
        std::string getRobotId() const;

        /**
         * getSessionId: returns a numerical id for the current session
         */
        u64 getSessionId() const;

        /**
         * getKeyframeId: returns a numerical id for the keyframe
         */
        u64 getKeyframeId() const;

        bool operator<(const LCD::Symbol& s1) {
            return (this->getRobotId() == s1.getRobotId()) && (this->getSessionId() == s1.getSessionId()) && (this->getKeyframeId() < s1.getKeyframeId());
        };

        bool operator>(const LCD::Symbol& s1) {
            return (this->getRobotId() == s1.getRobotId()) && (this->getSessionId() == s1.getSessionId()) && (this->getKeyframeId() > s1.getKeyframeId());
        };

        friend std::ostream& operator<<(std::ostream& os, const Symbol& s);
    private:
        u64 data = 0;
        std::string robot_id_ = "";
        u64 keyframe_id_ = 0;
        u64 session_id_ = 0;
    };
} // namespace LCD


#endif
