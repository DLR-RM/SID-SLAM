//
// Created by giub_ri on 9/20/21.
//

#ifndef VISUAL_LCD_LCDETECTOR_H
#define VISUAL_LCD_LCDETECTOR_H

#include "symbol.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Dense>

#include "srrg_hbst/types/binary_tree.hpp"

#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>

#define INFO(x) LCDetectorBase::printout(x, LCD::Verbosity::INFO)
#define WARN(x) LCDetectorBase::printout(x, LCD::Verbosity::WARN)
#define ERROR(x) LCDetectorBase::printout(x, LCD::Verbosity::ERROR)
#define ALWAYS(x) LCDetectorBase::printout(x, LCD::Verbosity::ALWAYS)


// Dummy
namespace LCD {
    
    // Utility functions 
    template<typename T> 
    T var(const std::vector<T>& vec) {
        const size_t sz = vec.size();
        if (sz == 1) {
            return 0.0;
        }
        // Calculate the mean
        const T mean = std::accumulate(vec.begin(), vec.end(), 0.0) / sz;
        // Now calculate the variance
        auto variance_func = [&mean, &sz](T accumulator, const T& val) {
            return accumulator + ((val - mean)*(val - mean) / (sz - 1));
        };
        return std::accumulate(vec.begin(), vec.end(), 0.0, variance_func); 
    };
    
	class Landmark {
	public:
		Landmark(const cv::Point2f& pt_und, const cv::Mat& descriptor,
				 const cv::Point3f& pt_3D) :
		  m_pt_und(pt_und), m_pt_3D(pt_3D) {
			m_descriptor = descriptor.clone();
			m_id = global_id++;

            if (!descriptor.empty())
			    m_initialized = true;
		};

		// Setters
		void setDescriptor(const cv::Mat& d) {
			m_descriptor = d.clone();
			m_initialized = true;
		}

		// Getters
		cv::Point2f feature() {return m_pt_und;}
		cv::Mat descriptor() {return m_descriptor.clone();}
		cv::Point3f point() {
			if (m_initialized) {
				return m_pt_3D;
			}
		}
		size_t id() {return m_id;}
		bool isInit() {return m_initialized;}

		float x() {return m_initialized ? m_pt_3D.x : 0;}
		float y() {return m_initialized ? m_pt_3D.y : 0;}
		float z() {return m_initialized ? m_pt_3D.z : 0;}

	private:
		cv::Point2f m_pt_und;
		cv::Mat m_descriptor;
		cv::Point3f m_pt_3D;

		bool m_initialized = false;

		size_t m_id = 0;
		static size_t global_id;
	};
}

typedef std::map<size_t, std::shared_ptr<LCD::Landmark>> LandmarkMap;
namespace LCD {
  	class Frame {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		/**
		 * constructor
		 * @param image					grayscale image
		 * @param K 					camera matrix
		 * @param d 					distortion coefficients
		 */
		Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat& d);

		/**
		* addPoint
		* @param keypoint_dist			2D location of the original feature, without correcting distortion
		* @param descriptor				1-row cv::Mat of the feature descriptor (32bit int for binary)
		* @param depth					depth of the point
		*/
		void addPoint(const cv::Point2f& keypoint_dist,
					  const cv::Mat& descriptor,
					  const float& depth);

		/**
		* addPoint
		* @param keypoint_dist			2D location of the original feature, without correcting distortion
		* @param descriptor				1-row cv::Mat of the feature descriptor (32bit int for binary)
		* @param landmark_coord			3D coordinates of the point in the keyframe reference frame
		*/
		void addPoint(const cv::Point2f& keypoint_dist,
					  const cv::Mat& descriptor,
					  const cv::Point3f& landmark_coord);

		/**
		 * compute
		 * extracts features and descriptors, depth is unknown and therefore will not
		 * be accounted for during validation. In general this should be avoided..
		 */
		void compute(const int& BITSIZE);

		// Getters
		size_t size() {return m_lmap.size();}
		bool isInit() {return is_initialized;}

		void getImage(cv::Mat& out) {out = m_I.clone();};
		void getKeypoints(std::vector<cv::Point2f>& out);
		void getDescriptors(cv::Mat& out);
		cv::Mat getCameraMatrix() {return m_K.clone();}
		float getFocalLenght() {return m_K.at<float>(0, 0);}

	protected:
		cv::Mat m_I = cv::Mat::zeros(1, 1, CV_8UC1);
		cv::Mat m_K = cv::Mat::zeros(3, 3, CV_32F);
		cv::Mat m_d = cv::Mat::zeros(5, 1, CV_32F);
		LandmarkMap m_lmap;
		bool is_initialized = false;
	};

	class Keyframe: public Frame {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		/**
		 * constructor
		 * @param image					grayscale image
		 * @param K 					camera matrix
		 * @param d 					distortion coefficients
		 * @param tf0					initial transformation for the keyframe (if unknown parse identity)
		 * @param index					unique index for the keyframe. should be in order of creation
		 */
		Keyframe(const cv::Mat& image, const cv::Mat& K, const cv::Mat& d,
				 const Eigen::Matrix4f& tf0, const size_t& index);

		// Getters
		size_t id() {return m_id;}
		const std::shared_ptr<LCD::Landmark>& getPoint(const size_t& id);

	private:
		size_t m_id;
		Eigen::Matrix4f m_tf = Eigen::Matrix4f::Identity();
	};
}

typedef std::map<size_t, std::shared_ptr<LCD::Keyframe>> KfMap;

namespace LCD {
  enum KFStrategy {
	  KF_RANGE_SEL, // selection of keyframes out of fixed range
	  KF_NOCOV_SEL  // selection of keyframes out of non-covisible set
  };

  class CandidateMatch {
  public:
	  size_t query_id;
	  size_t database_id;
	  std::vector<std::pair<cv::Point2f, cv::Point2f>> matched_pts;
  };

  class Result: public CandidateMatch {
  public:
	  size_t n_inliers = 0;
	  size_t n_tot_matches = 0;
	  float RMSE = 0.0;
	  float dT = 0.0;
	  float dT_depths = 0.0;
	  Eigen::Matrix4f tf_q_db; // should be the tf that brings query to database?
	  Eigen::Matrix<float, 6, 1> cov_q_db;
  };

  enum Verbosity {
	  INFO,
	  WARN,
	  ERROR,
	  ALWAYS,
	  NONE,
  };

  class LCDOptions {
  public:

	  // Visualize
	  bool visualize_matches = false;
	  bool write_match_images = false;

	  // Set verbosity to level INFO
	  bool is_verbose = false;

	  // Parameters related to keyframe blacklisting
	  LCD::KFStrategy keyframe_selection_strategy = LCD::KFStrategy::KF_RANGE_SEL;
	  int n_keyframe_nomatch = 100;

	  // HBST: Hamming distance to group descriptors as same visual word
	  // (relative to BITSIZE)
	  float relative_desc_dist = 0.1;

	  // HBST: Minimum number of matchables with database image to accept candidate
	  // (relative to number of extracted features)
	  float match_ratio = 0.01;

	  // Maximum Hamming distance between descriptors to filter matches after
	  // brute-force matching. Expressed as a percentage of the BITSIZE
	  float max_hamming_distance_to_reject_match = .4;

	  // Max depth of landmarks to use for match validation
	  float max_feature_depth = 50; // meters

	  // PnPRansac options
	  int pnp_max_iterations = 500;
	  float pnp_max_epipolar_error = 4.0; // px
	  float pnp_confidence = 0.999; // px

	  // Minimum inlier fraction after RANSAC to proceed with the validation
	  float min_inlier_ratio = 0.4;

	  // Max difference of z-coordinate (~depth) of aligned 3D points
	  float max_depth_difference_after_alignment = 0.1; // TODO: make it relative to depth

	  // Minimum ratio of aligned point pairs that satisfie the depth check
	  float min_ratio_points_depth_check = 0.7;
      
      // Monte-Carlo iterations for approximating uncertainty 
      int n_MC_iterations = 5;

	  // Dump stats to disk
	  bool dump_stats = false;
  };
}

class LCDetectorBase {
public:
	static void printout(const std::string& message, const LCD::Verbosity& verbosity = LCD::Verbosity::INFO);
	static LCD::Verbosity verbosity_level;
};

template<int BITSIZE>
class LCDetector: public LCDetectorBase {
	friend LCD::Keyframe;
public:
	/**
	 * LCDetector constructor for the LCDetector class
	 * @param strategy 			keyframe exclusion strategy from most similar candidates
	 * @param n_nomatch 		if range-based strategy, number of keyframes to exclude
	 */
	LCDetector(const LCD::LCDOptions& options);
	~LCDetector();

	/**
	 * parse keyframe, store pointer in keyframe map, add index to queue of un-processed keyframes
	 * @param kf				shared_ptr to LCD::Keyframe
	 */
	void parseKeyframe(const std::shared_ptr<LCD::Keyframe>& kf, const std::list<size_t>& covis_idxs = {});

	/**
	 * emergency relocalization. Assume here tracking lost, therefore the frame is not connected to anything else.
	 * This function blocks the processing and validation threads and queries immediately the database for a match.
	 * This frame is not added to the database.
	 * Note that it is the user's responsibility to properly maintain keyframe indexes for those who come later after
	 * a successful relocalization!
	 * @param f					shared_ptr to LCD::Keyframe
	 */
	bool localizeFrame(const std::shared_ptr<LCD::Frame>& f, LCD::Result& result);

	/**
	 * loopDetected: Ask LCDetector for validated loops and return list of results
	 * @param results			vector of result details (ids, tf)
	 * @return 					true if n>=1 loop detected
	 */
	 bool loopDetected(std::vector<LCD::Result>& results);

	 void setVerbosity(const LCD::Verbosity& v) {verbosity_level = v;}

protected:
	 bool searchMatches(const std::vector<cv::Point3f>& points_query,
	 				 	const cv::Mat& desc_query,
	 				 	const std::vector<cv::Point2f>& features_query,
					 	const cv::Mat& desc_db,
					 	const cv::Mat& K_query,
						const cv::Mat& K_db,
						const cv::Mat& rvec,
						const cv::Mat& tvec,
						std::vector<cv::DMatch>& matches);
private:
	/*
	 * worker function, runs in a thread. Checks for new keyframes, optionally computes features if not yet parsed,
	 * then queries the tree and pushes indexes to a match_queue
	 */
	void worker_process();

	/*
	 * worker function, runs in a thread. Checks for new candidate keyframe pair, performs validation and
	 * removes entries from the queue
	 */
	void worker_validation();

	/*
	 * Given two candidate keyframes, validate the match and return a tf
	 */
	bool validateMatch(const size_t& id0, const size_t& id1, LCD::Result& res, bool visualize = false);


	void shutdown() {isShutdown = true;}

	KfMap m_kmap;

	std::thread m_worker_process_thread;
	std::thread m_worker_validation_thread;

	std::list<size_t> m_process_queue = {};
	std::map<float, LCD::CandidateMatch> m_validation_queue = {};
	std::mutex m_process_mutex, m_validation_mutex;

	size_t BYTESIZE;

	LCD::LCDOptions m_options;
	LCD::KFStrategy m_strategy = LCD::KFStrategy::KF_RANGE_SEL;
	std::map<size_t, std::list<size_t>> m_no_match_list;
	size_t m_no_match_range = 0;

	float m_tree_ratio = 0.1;
	float m_match_ratio = 0.05;

	std::vector<LCD::Result> m_results_queue, m_results_record;

	bool isShutdown = false;

	bool m_visualize = false;

	using Matchable = srrg_hbst::BinaryMatchable<cv::Point2f, BITSIZE>; // matchable is a keypoint with binary descriptor of length 256 bits
	typedef srrg_hbst::BinaryNode<Matchable> Node;
	typedef srrg_hbst::BinaryTree<Node> Tree;
	std::unique_ptr<Tree> p_tree;

	std::shared_ptr<std::ofstream> m_stats_dump;
};

// Force compile of certain types
template class LCDetector<256>; // ORB - BRIEF
template class LCDetector<486>; // A-KAZE
template class LCDetector<512>; // BRISK

// Typedefs for convenience
typedef LCDetector<256> LCDetectorORB;
typedef LCDetector<486> LCDetectorAKAZE;
typedef LCDetector<512> LCDetectorBRISK;

#endif //VISUAL_LCD_LCDETECTOR_H
