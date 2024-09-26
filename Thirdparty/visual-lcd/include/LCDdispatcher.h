#ifndef LCD_DISPATCHER_H
#define LCD_DISPATCHER_H

class LCDdispatcher {
public:
	LCDdispatcher(int bitsize, const LCD::LCDOptions& options):
		bitsize_(bitsize) {

		bool verbose = options.is_verbose;
		switch (bitsize_) {
			case 256:
				d256.reset(new LCDetector<256>(options));
				LCDetector<256>::verbosity_level = verbose ? LCD::Verbosity::INFO : LCD::Verbosity::WARN;
				break;
			case 486:
				d486.reset(new LCDetector<486>(options));
				LCDetector<486>::verbosity_level = verbose ? LCD::Verbosity::INFO : LCD::Verbosity::WARN;
				break;
			case 512:
				d512.reset(new LCDetector<512>(options));
				LCDetector<512>::verbosity_level = verbose ? LCD::Verbosity::INFO : LCD::Verbosity::WARN;
				break;
			default:
				std::cout << "BITSIZE(" << bitsize_ << ") not supported" << std::endl;
				break;
		}
	};

	// Shadow here functions of LCDetector
	void parseKeyframe(const std::shared_ptr<LCD::Keyframe>& kf,
		const std::list<size_t>& covis_idxs = {}) {
		switch (bitsize_) {
			case 256:
				d256->parseKeyframe(kf, covis_idxs);
				break;
			case 486:
				d486->parseKeyframe(kf, covis_idxs);
				break;
			case 512:
				d512->parseKeyframe(kf, covis_idxs);
				break;
			default:
				std::cout << "BITSIZE(" << bitsize_ << ") not supported" << std::endl;
				break;
		}
	};

	bool loopDetected(std::vector<LCD::Result>& results) {
		switch (bitsize_) {
			case 256:
				return d256->loopDetected(results);
			case 486:
				return d486->loopDetected(results);
			case 512:
				return d512->loopDetected(results);
			default:
				std::cout << "BITSIZE(" << bitsize_ << ") not supported" << std::endl;
				return false;
		}
	};

private:
	int bitsize_;
	std::shared_ptr<LCDetector<256>> d256;
	std::shared_ptr<LCDetector<486>> d486;
	std::shared_ptr<LCDetector<512>> d512;
};

#endif
