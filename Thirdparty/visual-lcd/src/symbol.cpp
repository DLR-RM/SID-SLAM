//
// Created by giub_ri 5 Nov 2021
//

#include "symbol.h"
#include <bitset>

// https://stackoverflow.com/a/37195527
u64 getbits(u64 value, u64 offset, u64 n) {
  const unsigned max_n = 64;
  if (offset >= max_n)
    return 0; /* value is padded with infinite zeros on the left */
  value >>= offset; /* drop offset bits */
  if (n >= max_n)
    return value; /* all  bits requested */
  const u64 mask = (1u << n) - 1; /* n '1's */
  return value & mask;
}


namespace LCD {
    Symbol::Symbol(const std::string& robot_id, const u64& keyframe_id, 
            const u64& session_id): robot_id_(robot_id), keyframe_id_(keyframe_id),
            session_id_(session_id) {

        // truncate to max 2 letters in robot_id 
        u64 robot_bits = 0;
        for (size_t pos = 0; pos < 2 && pos < robot_id.size(); pos++) {
            robot_bits = robot_bits | (static_cast<u64>(robot_id.c_str()[pos]) << pos*8);
        }

        // Shift integers such that data = [16bit session - 16bit robot - 32bit keyframe]
        data = keyframe_id | (robot_bits << 32) | (session_id << 48);
    };

    Symbol::Symbol(const u64& _data): data(_data) {
        // Unpack
        robot_id_.append(1, char(getbits(data, 32, 8)));
        robot_id_.append(1, char(getbits(data, 40, 8)));
        session_id_ = getbits(data, 48, 16);
        keyframe_id_ = getbits(data, 0, 31);
    };

    std::ostream& operator<<(std::ostream& os, const Symbol& s) {
        os << "(" << s.robot_id_  << ", " << s.keyframe_id_ << ", " << s.session_id_ << ")";
        return os;
    }

    u64 Symbol::getNumeric() const {
        return data;
    };

    std::string Symbol::getRobotId() const {
        return robot_id_;
    }

    u64 Symbol::getKeyframeId() const {
        return keyframe_id_;
    }

    u64 Symbol::getSessionId() const {
        return session_id_;
    }
}
