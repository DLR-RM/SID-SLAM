//
// Created by giub_ri on 9/20/21.
//

#include "LCDetector.h"
#include <random>
#include <algorithm>


// Forward declare some functions
Eigen::Matrix4f cvRt2eigen(cv::Mat& R, cv::Mat& t);
cv::Mat eigen2mat(const Eigen::Matrix4f& input);
void computeReprojectionError(const std::vector<cv::Point2f>& features, const std::vector<cv::Point3f>& points,
							  const cv::Mat& R, const cv::Mat& t, const cv::Mat& K, std::vector<float>& err, float& rmse);

size_t LCD::Landmark::global_id = 0;

LCD::Verbosity LCDetectorBase::verbosity_level = LCD::Verbosity::INFO;

LCD::Frame::Frame(const cv::Mat& image, const cv::Mat& K, const cv::Mat& d) {
	m_I = image.clone();
	m_K = K.clone();
	m_d = d.clone();
	m_K.convertTo(m_K, CV_32FC1);
	m_d.convertTo(m_d, CV_32FC1);
}

LCD::Keyframe::Keyframe(const cv::Mat& image, const cv::Mat& K, const cv::Mat& d,
				 const Eigen::Matrix4f& tf0, const size_t& index):
				 Frame(image, K, d) {
	m_tf = tf0;
	m_id = index;
}

const std::shared_ptr<LCD::Landmark>& LCD::Keyframe::getPoint(const size_t& id) {
	if (m_lmap.count(id) == 0) {
		std::cout << "[ ERROR] in Keyframe::getPoint(): wanted landmark with local id: " <<
			id << " (in keyframe " << m_id << ") which does not exist. n_landmarks: " <<
			m_lmap.size() << std::endl;
		return nullptr;
	} else {
		return m_lmap.at(id);
	}
};

void LCD::Frame::compute(const int& BITSIZE) {

	if (is_initialized) {
		return;
	}

	switch (BITSIZE) {
		case 256: // ORB default
			{
				cv::Ptr<cv::DescriptorExtractor> descriptor_extractor = cv::ORB::create(300, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 10);
				for (auto& l:m_lmap) {
					if (l.second->descriptor().empty()) {
						std::vector<cv::KeyPoint> keypoint = {cv::KeyPoint(l.second->feature(), 31)};
						cv::Mat descriptor;
						descriptor_extractor->compute(m_I, keypoint, descriptor);
						l.second->setDescriptor(descriptor);
					}
				}
				is_initialized = true;
			}
			break;
		default:
			WARN("in Keyframe::compute(): required computing descriptors with bitsize " + std::to_string(BITSIZE) +
				" but this is not implemented. ");
			break;
	}
}

void LCD::Frame::addPoint(const cv::Point2f& keypoint_dist,
							 const cv::Mat& descriptor,
							 const float& depth) {

	if (depth < 1e-6) return;

	std::vector<cv::Point2f> features = {keypoint_dist};
	std::vector<cv::Point2f> features_und;
	cv::undistortPoints(features, features_und, m_K, m_d, cv::noArray(), m_K);

	cv::Point3f pt3d;
 	pt3d.x = (features_und.at(0).x - m_K.at<float>(0, 2)) / m_K.at<float>(0, 0) * depth;
 	pt3d.y = (features_und.at(0).y - m_K.at<float>(1, 2)) / m_K.at<float>(1, 1) * depth;
 	pt3d.z = depth;

	std::shared_ptr<LCD::Landmark> p_landmark(
		new Landmark(features_und.at(0), descriptor, pt3d));

	size_t local_id = m_lmap.size();

	if (m_lmap.count(local_id) > 0) {
		WARN("in Keyframe::addPointUnd(): tried to insert a landmark with an already existing id. Returning");
		return;
	} else {
		m_lmap[local_id] = p_landmark;
	}
	is_initialized = true;
};

void LCD::Frame::addPoint(const cv::Point2f& keypoint_dist,
							 const cv::Mat& descriptor,
							 const cv::Point3f& landmark_coord) {
	std::vector<cv::Point2f> features = {keypoint_dist};
	std::vector<cv::Point2f> features_und;
	cv::undistortPoints(features, features_und, m_K, m_d, cv::noArray(), m_K);

	std::shared_ptr<LCD::Landmark> p_landmark(
		new Landmark(features_und.at(0), descriptor, landmark_coord));

	size_t local_id = m_lmap.size();

	if (m_lmap.count(local_id) > 0) {
		WARN("in Keyframe::addPointUnd(): tried to insert a landmark with an already existing id. Returning");
		return;
	} else {
		m_lmap[local_id] = p_landmark;
	}
	is_initialized = true;
};

void LCD::Frame::getKeypoints(std::vector<cv::Point2f>& out) {
	out.clear();
	for (const auto& p:m_lmap) {
		out.push_back(p.second->feature());
	}
}

void LCD::Frame::getDescriptors(cv::Mat& out) {
	out.release();
	size_t cnt = 0;
	for (const auto& p:m_lmap) {
		if (p.second->descriptor().empty()) {
			ERROR("Empty descriptor in landmark");
			return;
		}
		if (cnt == 0) {
			out = cv::Mat(this->size(), p.second->descriptor().cols, CV_8UC1);
			p.second->descriptor().copyTo(out.row(0));
			cnt++;
			continue;
		}
		if (p.second->descriptor().cols != out.cols) {
			std::stringstream ss;
			ss << "in Keyframe::getDescriptors(): mismatched descriptor size. " <<
			 p.second->descriptor().cols << " vs " << out.cols << " Is the descriptor definition correct?";
			ERROR(ss.str());
			return;
		}
		p.second->descriptor().copyTo(out.row(cnt));
		cnt++;
	}
}

template<int BITSIZE>
LCDetector<BITSIZE>::LCDetector(const LCD::LCDOptions& options):
  m_options(options),
  m_strategy(options.keyframe_selection_strategy),
  m_no_match_range(options.n_keyframe_nomatch),
  BYTESIZE(size_t(std::ceil(float(BITSIZE) / 8.0))) {

	m_tree_ratio = options.relative_desc_dist;
	m_match_ratio = options.match_ratio;
	m_visualize = options.visualize_matches;

	INFO("Created LCDetector. Descriptor size in bytes: " + std::to_string(BYTESIZE));
	p_tree.reset(new srrg_hbst::BinaryTree<Node>(0));

	m_worker_process_thread = std::thread(&LCDetector<BITSIZE>::worker_process, this);
	m_worker_validation_thread = std::thread(&LCDetector<BITSIZE>::worker_validation, this);

	if (m_options.dump_stats) {
		m_stats_dump.reset(new std::ofstream("/tmp/stats_dump.txt"));
	}
}

template<int BITSIZE>
LCDetector<BITSIZE>::~LCDetector() {
	this->shutdown();
	m_worker_process_thread.join();
	m_worker_validation_thread.join();
	if (m_stats_dump) {
		m_stats_dump->close();
	}
}

template<int BITSIZE>
void LCDetector<BITSIZE>::worker_process() {
	using namespace std::literals::chrono_literals;

	while (!isShutdown) {
		{
			const std::lock_guard<std::mutex> lock(m_process_mutex);

			// Empty queue
			while (!m_process_queue.empty()) {

				auto start = std::chrono::steady_clock::now();

				const size_t query_id = m_process_queue.front();
				m_process_queue.pop_front();

				if (m_kmap.count(query_id) == 0) {
					std::stringstream ss;
					ss << "in worker_process(): trying to access keyframe id: " << query_id << " but is not in map";
					WARN(ss.str());
					continue;
				}

				// If added keyframe is not initialized, do it here
				if (!m_kmap.at(query_id)->isInit()) {

					// Compute here with descriptor that depends on the bitsize!
					m_kmap.at(query_id)->compute(BITSIZE);
				}

				// Check here against older keyframes?
				std::vector<cv::Point2f> pts;
				cv::Mat desc;
				m_kmap.at(query_id)->getKeypoints(pts);
				m_kmap.at(query_id)->getDescriptors(desc);

				// Check
				if (pts.empty()) {
					std::stringstream ss;
					ss << "Empty keypoints for keyframe id: " << query_id;
					INFO(ss.str());
				}
				if (pts.size() != desc.rows) {
					std::stringstream ss;
					ss << "In LCDetector::worker_process(): Different size of descriptors (" <<
					 	desc.rows << ") and keypoints (" << pts.size() << ") for keyframe id: " << query_id;
					ERROR(ss.str());
				}

				const typename Tree::MatchableVector matchables(
					Tree::getMatchables(
						desc,  // cv::Mat nx32 (if 256 bits)
						pts,    // opencv
						m_kmap.at(query_id)->id()));

				typename Tree::MatchVectorMap matches_per_image, matches_per_image_filt;

				p_tree->matchAndAdd(matchables, matches_per_image, m_tree_ratio * float(BITSIZE)); // .1 * bitsize

				// Filter matches given keyframe selection strategy. Exclude blacklisted keyframes from match list
				std::list<size_t> no_match_for_current_kf;
				try {
					no_match_for_current_kf = m_no_match_list.at(query_id);
				} catch (std::exception& e) {
					std::stringstream ss;
					ss << "in worker_process: " << e.what();
					WARN(ss.str());
					continue;
				}

				for(const auto& m:matches_per_image) {                      // Loop between candidate keyframe matched to the query
					bool is_match_allowed = true;
					for (const auto& no_match_id:no_match_for_current_kf) { // Loop between indexes of blacklisted keyframes to this query
                        if (no_match_id == m.first ||                       // Is this candidate blacklisted?
                            LCD::Symbol(m.first) > LCD::Symbol(query_id))   // newer keyframes added before emptying the queue
                            is_match_allowed = false;
					}

					if (is_match_allowed) {
						matches_per_image_filt.insert(m);
					}
				}

				if (matches_per_image_filt.empty())
					continue;

				// Loop over match vector for each image in the past
				size_t image_id_max, n_matches;
				size_t max_matches = 0;

				for (const auto& m:matches_per_image_filt) {
					const size_t database_id = m.first;
					if (m.second.size() >= max_matches) {
						max_matches = m.second.size();
						image_id_max = database_id;
					}
				}

				// Check
				if (m_kmap.count(image_id_max) == 0 ||
			  		matches_per_image_filt.count(image_id_max) == 0) {
					std::stringstream ss;
					ss << "in LCDetector::worker_process(), kf id: " << image_id_max << " not in database";
					INFO(ss.str());
					continue;
				}

				bool is_candidate_found = false;

				// heuristic num. 1: fraction of matches
				size_t n_matchable_total = m_kmap.at(query_id)->size() + m_kmap.at(image_id_max)->size();
				float match_fraction = 2 * float(matches_per_image_filt.at(image_id_max).size()) / float(n_matchable_total);
				bool is_enough_matches = match_fraction > m_match_ratio ? true : false;

				// heuristic num. 2: consistent "scores" for neighbors
				bool is_match_temporally_consistent = false;
				for (const auto& m:matches_per_image_filt) {
					const size_t database_id = m.first;
					if (m_kmap.count(database_id) == 0) {
                        std::stringstream ss;
                        ss << "in LCDetector::worker_process(): no entry in database for id: " << database_id;
                        WARN(ss.str());
                        continue; // ??
                    }

                    LCD::Symbol s_db(database_id);
					LCD::Symbol s_max(image_id_max);

					//if (database_id == image_id_max-1 || database_id == image_id_max+1) {
					if (s_db.getRobotId() == s_max.getRobotId() &&
						(s_db.getKeyframeId() == s_max.getKeyframeId()-1 || s_db.getKeyframeId() == s_max.getKeyframeId()+1)) {
						size_t n_matches_neighbors = matches_per_image_filt.at(database_id).size();
						size_t n_matchable_total_neighbors = m_kmap.at(query_id)->size() + m_kmap.at(database_id)->size();
						if (2 * float(n_matches_neighbors) / float(n_matchable_total_neighbors) > .5 * match_fraction) {
							is_match_temporally_consistent = true;
						} else {
							is_match_temporally_consistent = false;
						}
					}
				}

				std::stringstream ss;
				ss << "in LCDetector::worker_process(): max corr: " << max_matches <<
						  " query_id: " << LCD::Symbol(query_id) << " database_id: " << LCD::Symbol(image_id_max) <<
						  " enough_matches: " << is_enough_matches << " temp_consistent: " << is_match_temporally_consistent;
			    INFO(ss.str());

				is_candidate_found = is_enough_matches; //&& is_match_temporally_consistent;

				// Add to validation queue
				if (is_candidate_found) {
					const std::lock_guard<std::mutex> lock(m_validation_mutex);
					LCD::CandidateMatch mc;
					mc.query_id = query_id;
					mc.database_id = image_id_max;
					for (const auto& m:matches_per_image_filt.at(image_id_max)) {
						mc.matched_pts.push_back(std::pair<cv::Point2f, cv::Point2f>(
							m.object_query, m.object_references[0]));
					}
					m_validation_queue.insert(std::pair<float, LCD::CandidateMatch>(match_fraction, mc));
				}

				auto stop = std::chrono::steady_clock::now();
				if (m_stats_dump) {
					*m_stats_dump << "[PROCESS_THREAD] " <<
						matches_per_image_filt.at(image_id_max).size() << " " <<
						size_t(std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count()) << std::endl;
				}

				if (m_visualize && true) {
					cv::Mat I_query, I_database, display, display_rgb, display_rgb_res;
					std::vector<cv::Point2f> keypoints_query, keypoints_database;

					m_kmap.at(query_id)->getImage(I_query);
					m_kmap.at(image_id_max)->getImage(I_database);
					m_kmap.at(query_id)->getKeypoints(keypoints_query);
					m_kmap.at(image_id_max)->getKeypoints(keypoints_database);


					cv::hconcat(I_query, I_database, display);
					cv::cvtColor(display, display_rgb, cv::COLOR_GRAY2BGR);
					cv::Point2f hshift(I_query.cols, 0);

					cv::resize(display_rgb, display_rgb_res, cv::Size(), 0.5, 0.5);

					if (is_candidate_found) {
						for (const auto& pt:keypoints_query) {
							cv::circle(display_rgb_res, .5*pt, 1, cv::Scalar(0, 100, 255), 1, 8);
						}
						for (const auto& pt:keypoints_database) {
							cv::circle(display_rgb_res, .5*pt + .5*hshift, 1, cv::Scalar(0, 100, 255), 1, 8);
						}
						for (const auto& m:matches_per_image_filt.at(image_id_max)) {
							cv::circle(display_rgb_res, .5*m.object_query, 1, cv::Scalar(0, 100, 255), 1, 8);
							cv::circle(display_rgb_res, .5*m.object_references[0] + .5*hshift, 2, cv::Scalar(0, 100, 255), 1, 8);
							cv::line(display_rgb_res,
									 .5*m.object_query,
									 .5*m.object_references[0] + .5*hshift,
									 cv::Scalar(0, 255, 0), 2, 8);
						}
					} else {
						for (const auto& pt:keypoints_query) {
							cv::circle(display_rgb_res, .5*pt, 1, cv::Scalar(0, 100, 255), 1, 8);
						}
						for (const auto& pt:keypoints_database) {
							cv::circle(display_rgb_res, .5*pt + .5*hshift, 1, cv::Scalar(0, 100, 255), 1, 8);
						}
						for (const auto& m:matches_per_image_filt.at(image_id_max)) {
							cv::line(display_rgb_res,
									 .5*m.object_query,
									 .5*m.object_references[0] + .5*hshift,
									 cv::Scalar(0, 0, 0), 2, 8);
						}
					}
					std::stringstream ss0, ss1, ss2, ss3;
					ss0 << "Loop candidate: (" << query_id << ", " << image_id_max << ")";
					ss1 << "n_matches: " << matches_per_image_filt.at(image_id_max).size();
					ss2 << "match_found: " << is_candidate_found;
					ss3 << "time: " << size_t(std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count()) << " ms";
					cv::putText(display_rgb_res, ss0.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
					cv::putText(display_rgb_res, ss1.str(), cv::Point(20, 35), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
					cv::putText(display_rgb_res, ss2.str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
					cv::putText(display_rgb_res, ss3.str(), cv::Point(20, 65), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
					cv::imshow("Loop Visualizer", display_rgb_res);
					if (m_options.write_match_images) {
						cv::imwrite("/tmp/match_" + std::to_string(query_id) + ".png", display_rgb_res);
					}
					cv::waitKey(20);
				}
			} // end queue loop
		} // end locked scope

		// Unlock and sleep
		std::this_thread::sleep_for(100ms);
	}
}

template<int BITSIZE>
void LCDetector<BITSIZE>::worker_validation() {
	using namespace std::chrono_literals;

	while (!isShutdown) {

		{
			std::unique_lock<std::mutex> lk1(m_process_mutex, std::defer_lock);
			std::unique_lock<std::mutex> lk2(m_validation_mutex, std::defer_lock);
			std::lock(lk1, lk2);

			auto start = std::chrono::steady_clock::now();

			if (!m_validation_queue.empty()) {

				// Empty queue from the back (higher match fraction / "score")
				std::map<float, LCD::CandidateMatch>::reverse_iterator rit;
				for (rit=m_validation_queue.rbegin(); rit!=m_validation_queue.rend(); ++rit) {
					LCD::Result res;
					if (this->validateMatch(rit->second.query_id, rit->second.database_id, res, m_visualize)) {
						res.query_id = rit->second.query_id;
						res.database_id = rit->second.database_id;
						res.matched_pts = rit->second.matched_pts; // from HBST db
						m_results_queue.push_back(res);
						m_results_record.push_back(res);

						INFO("Validated match between query: " + std::to_string(res.query_id) + " database: " + std::to_string(res.database_id));
						INFO("n_inliers: " + std::to_string(res.n_inliers) + " / " + std::to_string(res.n_tot_matches) + " RMSE: " + std::to_string(res.RMSE));
					}
				}

				m_validation_queue.clear();
			}

			auto stop = std::chrono::steady_clock::now();
			if (m_stats_dump) {
				*m_stats_dump << "[VALIDATION_THREAD] " <<
					m_validation_queue.size() << " " <<
			  		size_t(std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count()) << std::endl;
			}
		} // end lock validation_mutex
		std::this_thread::sleep_for(100ms);
	}
}

template<int BITSIZE>
bool LCDetector<BITSIZE>::loopDetected(std::vector<LCD::Result>& results) {
	const std::lock_guard<std::mutex> lock(m_validation_mutex);
	if (m_results_queue.empty()) {
		return false;
	}

	results = m_results_queue;
	m_results_queue.clear();
	return true;
}

template<int BITSIZE>
void LCDetector<BITSIZE>::parseKeyframe(const std::shared_ptr<LCD::Keyframe>& kf, const std::list<size_t>& covis_idxs) {
	std::unique_lock<std::mutex> lock(m_process_mutex);

	// Insert new keyframe into map
	if (m_kmap.count(kf->id()) > 0) {
		LCDetector<BITSIZE>::printout("[WARN] in parseKeyframe: tried to insert a keyframes with an already existing id. Returning",
			LCD::Verbosity::INFO);
		return;
	}
	m_kmap[kf->id()] = kf;

    LCD::Symbol s_curr(kf->id());
	switch (m_strategy) {
		case LCD::KFStrategy::KF_RANGE_SEL:
			for (const auto& k:m_kmap) {
				//std::unique_lock<std::mutex> lock(m_process_mutex);
                LCD::Symbol s_db(k.first);
				if ( int(s_db.getKeyframeId()) > ( int(s_curr.getKeyframeId()) - int(m_no_match_range)))
					if (m_no_match_list.count(kf->id()) == 0) {
						m_no_match_list[kf->id()] = std::list<size_t>({k.first});
					} else {
						m_no_match_list.at(kf->id()).push_back(k.first);
                        LCD::Symbol tmp(k.first);
					}
			}
			break;
		case LCD::KFStrategy::KF_NOCOV_SEL:
			if (covis_idxs.empty()) {
				std::cout << "\033[1;31m[ ERR]\033[0m In LCDetector::parseKeyframe(): list of covisible idxs can not be empty if strategy KF_NOCOV_SEL is selected" << std::endl;
				return;
			}
			{
				//std::unique_lock<std::mutex> lock(m_process_mutex);
				m_no_match_list[kf->id()] = covis_idxs;
			}
			break;
		default:
			std::cout << "\033[1;31m[ ERR]\033[0m: in LCDetector::parseKeyframe(): unknown strategy: " << m_strategy << std::endl;
			return;
	}

	// Lock the queue, m_kmap is only growing and therefore concurrent readings should not screw up..
	{
		// std::unique_lock<std::mutex> lock(m_process_mutex);
		// m_no_match_list.at(kf->id()).sort();
		m_process_queue.push_back(kf->id());
	}

	std::stringstream ss;
	ss << "parsed Keyframe id: " << kf->id() << " to LCDetector" ;
	INFO(ss.str());
};

template<int BITSIZE>
bool LCDetector<BITSIZE>::validateMatch(const size_t& id0, const size_t& id1, LCD::Result& res, bool visualize) {

	if (m_kmap.count(id0) == 0 || m_kmap.count(id1) == 0) {
		ERROR("in LCDetector::validateMatch(): matching id0: " + std::to_string(id0) + " and id1: " + std::to_string(id1) +
			" but at least one of them is not pointing to an existing keyframe");
		return false;
	}

	// Brute Force match
	std::vector<cv::DMatch> matches, matches_filt;

	cv::Mat desc0, desc1;
	cv::Mat K0, K1;
	float focal_kf0, focal_kf1;

	try {
		m_kmap.at(id0)->getDescriptors(desc0);
		m_kmap.at(id1)->getDescriptors(desc1);
		K0 = m_kmap.at(id0)->getCameraMatrix();
		K1 = m_kmap.at(id1)->getCameraMatrix();
		focal_kf0 = m_kmap.at(id0)->getFocalLenght();
		focal_kf1 = m_kmap.at(id1)->getFocalLenght();
	} catch (std::exception& e) {
		std::cout << "in validateMatch: " << e.what() << std::endl;
	}

	std::shared_ptr<cv::BFMatcher> matcher_(new cv::BFMatcher(cv::NORM_HAMMING, true));
	matcher_->match(desc0, desc1, matches);

	// If zero matches return
	if(matches.size() < 1) {
		INFO("Validation: zero matches");
		return false;
	}

	std::vector<char> status;
	int good = 0;

	for(size_t m=0; m<matches.size(); m++) {
		if(matches.at(m).distance <
			(float) BITSIZE * m_options.max_hamming_distance_to_reject_match) {
			status.push_back(1);
			matches_filt.push_back(matches.at(m));
			good++;
		} else {
			status.push_back(0);
		}
	}

	// If zero matches return
	if(matches_filt.size() < 1) {
		std::vector<char> temp_have_depth2(matches_filt.size(), true);
		INFO("Validation: zero matches");
		return false;
	}

	// could use P3P ransac of whatnot. keep ransac as a first step..
	cv::Mat r12, t12;
	cv::Mat inl;

	std::vector<cv::Point3f> points3D_align0, points3D_align1;
	std::vector<cv::Point2f> keypoints_align0, keypoints_align1;
	std::vector<cv::Point3f> points3D_all_align0, points3D_all_align1;
	std::vector<cv::Point2f> keypoints_all_align0, keypoints_all_align1;
	points3D_align0.reserve(1000);
	keypoints_align1.reserve(1000);

	// Select keypoints for PNP
	size_t count_have_depth = 0;
	std::vector<int> match_ids0, match_ids1;
	for(size_t k=0; k<matches_filt.size(); k++) {

		// Check if there are zero depths
		if (m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->isInit() == 0 ||
			m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->isInit() == 0 ||
			m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->z() > m_options.max_feature_depth ||
			m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->z() > m_options.max_feature_depth) {

			keypoints_all_align1.push_back(m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->feature());
			keypoints_all_align0.push_back(m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->feature());
			continue;
		}

		keypoints_all_align1.push_back(m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->feature());
		keypoints_all_align0.push_back(m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->feature());

		points3D_align0.push_back(m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->point());
		keypoints_align1.push_back(m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->feature());
		points3D_align1.push_back(m_kmap.at(id1)->getPoint(matches_filt.at(k).trainIdx)->point());
		keypoints_align0.push_back(m_kmap.at(id0)->getPoint(matches_filt.at(k).queryIdx)->feature());

		match_ids0.push_back(matches_filt.at(k).queryIdx);
		match_ids1.push_back(matches_filt.at(k).trainIdx);

		count_have_depth++;
	}

	// Get all features that are not matched originally
	std::vector<cv::Point3f> points3D_unmatched0, points3D_unmatched1;
	std::vector<cv::Point2f> keypoints_unmatched0, keypoints_unmatched1;
	cv::Mat desc_unmatched0(0, BYTESIZE, CV_8UC1);
	cv::Mat desc_unmatched1(0, BYTESIZE, CV_8UC1);
	for (size_t i=0; i<desc0.rows; i++) {
		if(std::find(match_ids0.begin(), match_ids0.end(), i) == match_ids0.end()) {
			if (m_kmap.count(id0) == 0 ||
				m_kmap.at(id0)->getPoint(i)->isInit() == 0 ||
				m_kmap.at(id0)->getPoint(i)->z() > m_options.max_feature_depth) continue;
			points3D_unmatched0.push_back(m_kmap.at(id0)->getPoint(i)->point());
			keypoints_unmatched0.push_back(m_kmap.at(id0)->getPoint(i)->feature());
			desc_unmatched0.push_back(desc0.row(i));
		}
	}
	for (size_t i=0; i<desc1.rows; i++) {
		if(std::find(match_ids1.begin(), match_ids1.end(), i) == match_ids1.end()) {
			if (m_kmap.count(id1) == 0 ||
				m_kmap.at(id1)->getPoint(i)->isInit() == 0 ||
				m_kmap.at(id1)->getPoint(i)->z() > m_options.max_feature_depth) continue;
			points3D_unmatched1.push_back(m_kmap.at(id1)->getPoint(i)->point());
			keypoints_unmatched1.push_back(m_kmap.at(id1)->getPoint(i)->feature());
			desc_unmatched1.push_back(desc1.row(i));
		}
	}

	bool usingEmat = false;
	if (count_have_depth < 5) {
#if 0
		INFO("Validation: zero matches after depth filtering. Validating using Fmat");

		std::vector<uchar> mask;
		cv::Mat E = cv::findEssentialMat(keypoints_all_align0, keypoints_all_align1, K0,
			cv::RANSAC, .999, 3, mask);
		int inliers = 0;
		for (const auto& i:mask) {
			if (i) inliers++;
		}

		// Check inliers
		if (float(inliers) / float(keypoints_all_align0.size()) < 0.4) {
			INFO("Visual match not validated after RANSAC: " + std::to_string(inliers) + " / " +
				std::to_string(keypoints_all_align0.size()) + " (Essential Mat) inliers");
			return false;
		}

		inliers = cv::recoverPose(E, keypoints_all_align0, keypoints_all_align1,
			K0, r12, t12);

		// Check inliers
		if (float(inliers) / float(keypoints_all_align0.size()) < 0.4) {
			INFO("Visual match not validated after RANSAC: " + std::to_string(inliers) + " / " +
				std::to_string(keypoints_all_align0.size()) + " (cheirality) inliers");
			return false;
		}

		tf = cvRt2eigen(r12, t12);
#endif
		return false;
	}

	cv::solvePnPRansac(
		points3D_align0,
		keypoints_align1,
		K1,
		cv::Mat(),
		r12,
		t12,
		false,
		m_options.pnp_max_iterations,
		m_options.pnp_max_epipolar_error,
		m_options.pnp_confidence,
		inl,
		cv::SOLVEPNP_P3P);

	if( (float) inl.rows / (float) points3D_align0.size() < m_options.min_inlier_ratio) {
		INFO("Visual match not validated after RANSAC: " + std::to_string(inl.rows) +
		 	" / " + std::to_string(points3D_align0.size()) + " inliers");
		return false;
	}

	// Refine RANSAC estimate with LM
	std::vector<cv::Point3f> points3D_align0_inl, points3D_align1_inl;
	std::vector<cv::Point2f> keypoints_align0_inl, keypoints_align1_inl;

	std::vector<cv::Point3f> points3D_align0_out, points3D_align1_out;
	std::vector<cv::Point2f> keypoints_align0_out, keypoints_align1_out;

	std::vector<int> inl_vector;
	for (size_t j=0; j<inl.rows; j++) {
		points3D_align0_inl.push_back(points3D_align0.at(inl.at<int>(j)));
		points3D_align1_inl.push_back(points3D_align1.at(inl.at<int>(j)));
		keypoints_align0_inl.push_back(keypoints_align0.at(inl.at<int>(j)));
		keypoints_align1_inl.push_back(keypoints_align1.at(inl.at<int>(j)));
		inl_vector.push_back(inl.at<int>(j));
	}

	// Get outlier indexes
	std::vector<int> outlier_idxs;
	for (size_t i=0; i<keypoints_align1.size(); i++) {
		if(std::find(inl_vector.begin(), inl_vector.end(), i) == inl_vector.end()) {
		    outlier_idxs.push_back(i);
			points3D_align0_out.push_back(points3D_align0.at(i));
			points3D_align1_out.push_back(points3D_align1.at(i));
			keypoints_align0_out.push_back(keypoints_align0.at(i));
			keypoints_align1_out.push_back(keypoints_align1.at(i));
		}
	}

	cv::Mat R12_beforeLM;
	cv::Rodrigues(r12, R12_beforeLM);
	R12_beforeLM.convertTo(R12_beforeLM, CV_32FC1);
	t12.convertTo(t12, CV_32FC1);

	std::vector<float> repr_err;
	float rmse;
	computeReprojectionError(keypoints_align1_inl, points3D_align0_inl, R12_beforeLM, t12, K0,
							 repr_err, rmse);
    
    // Should not happen, but just to be safe, see iss_9
    if (rmse > m_options.pnp_max_epipolar_error) {
        INFO("in LCDetector::validateMatch(): rejected match as RMSE > pnp_max_epipolar_error");
        return false;
    }

	Eigen::Matrix4f T12_beforeLM = cvRt2eigen(R12_beforeLM, t12);

	// Approximate uncertainty from MC test with very sparse sample (n=5 be default, otherwise too comp. intensive)
	const double mean = 0.0;
	const double stddev = 1; // px
	std::default_random_engine gen;
	std::normal_distribution<double> dist(mean, stddev);
    float rmse_noise;
    float delta_t_depths = 0.0;
    float delta_t = 0.0;
    Eigen::Matrix<float, 6, 1> cov;
    
    std::vector<float> eul_1_MC, eul_2_MC, eul_3_MC, x_MC, y_MC, z_MC; 
    for (int MC_it = 0; MC_it < m_options.n_MC_iterations; MC_it++) {
        
        std::vector<cv::Point2f> keypoints_align1_inl_noise;
        for (const auto& p:keypoints_align1_inl) {
            keypoints_align1_inl_noise.push_back(p + cv::Point2f(dist(gen), dist(gen)));
        }

        cv::Mat r12_noise, t12_noise;
        cv::solvePnPRansac(
            points3D_align0_inl,
            keypoints_align1_inl_noise,
            K1,
            cv::Mat(),
            r12_noise,
            t12_noise,
            false,
            m_options.pnp_max_iterations,
            m_options.pnp_max_epipolar_error,
            m_options.pnp_confidence,
            inl,
            cv::SOLVEPNP_P3P);

        cv::Mat R12_noise;
        cv::Rodrigues(r12_noise, R12_noise);
        R12_noise.convertTo(R12_noise, CV_32FC1);
        t12_noise.convertTo(t12_noise, CV_32FC1);

        std::vector<float> repr_noise;
        computeReprojectionError(keypoints_align1_inl_noise, points3D_align0_inl, R12_noise, t12_noise, K0,
                                repr_noise, rmse_noise);
        Eigen::Matrix4f T12_noise = cvRt2eigen(R12_noise, t12_noise);

        // Check error
        // Simple hack, as Eigen::eulerAngles are arbotrary to +-pi, compute rotation from original solution to the one 
        // with noise (as to get the deffs. ) and combine with small rotation of 10 degrees. Then get Euler angles and 
        // compute the variance.
        
        Eigen::Matrix4f T12_original = cvRt2eigen(R12_beforeLM, t12);
        Eigen::Matrix4f T_err_centered = T12_original.inverse() * T12_noise;
        
        Eigen::AngleAxisf rAngle(.17, Eigen::Vector3f::UnitZ()); // .17 rad ~= 10 deg
        Eigen::AngleAxisf yAngle(.17, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf pAngle(.17, Eigen::Vector3f::UnitX());
        Eigen::Quaternion<float> q = rAngle * yAngle * pAngle;
        Eigen::Matrix3f offset_rotation_mat = q.matrix() * T_err_centered.block<3,3>(0,0);
        
        Eigen::Vector3f ea = offset_rotation_mat.eulerAngles(0, 1, 2).cast<float>(); 
        Eigen::Vector3f et; et << T_err_centered(0,3), T_err_centered(1,3), T_err_centered(2,3);
        
        eul_1_MC.push_back(ea(0)); 
        eul_2_MC.push_back(ea(1)); 
        eul_3_MC.push_back(ea(2)); 
        x_MC.push_back(et(0));  
        y_MC.push_back(et(1)); 
        z_MC.push_back(et(2));
        
        // Compute translation error relative to the mean depth
        float depth = 0.0;
        for (const auto& p:points3D_align0_inl) {
            depth += p.z;
        }
        depth /= float(points3D_align0_inl.size());
        delta_t = et.norm();
        delta_t_depths = delta_t / depth;
    }
    
    if (m_options.n_MC_iterations > 1) {
        cov << LCD::var(x_MC), LCD::var(y_MC), LCD::var(z_MC),
               LCD::var(eul_1_MC), LCD::var(eul_2_MC), LCD::var(eul_3_MC); 
    } else {
        cov << 0.0001, 0.0001, 0.0001, 0.0005, 0.0005, 0.0005;
    }
    
    if (cov(3) > 0.03 || cov(4) > 0.03 || cov(5) > 0.03 || // 10 degrees
        cov(0) > 0.01 || cov(1) > 0.01 || cov(2) > 0.01) { // 10 centimeters
        WARN("in LCDetector::validateMatch(): rejected match, unc. > 10 deg or 10 cm");
        return false;
    }

	// Align keyframes and verify that depths are consistent.
	std::vector<float> z_offsets;
	z_offsets.reserve(points3D_align0_inl.size());

	float thresh = m_options.max_depth_difference_after_alignment; // [m]
	float perc   = m_options.min_ratio_points_depth_check; // [%]
	int counter_ok = 0;

	float baseline = 0.1; // TODO: parse later
	float sigma_px = 0.5; // normally at least in the center

	// std::cout << "Offsets: ";
	for (size_t j=0; j<points3D_align0_inl.size(); j++) {
		float depth_0 =
			R12_beforeLM.at<float>(2,0) * (float) points3D_align0_inl.at(j).x +
			R12_beforeLM.at<float>(2,1) * (float) points3D_align0_inl.at(j).y +
			R12_beforeLM.at<float>(2,2) * (float) points3D_align0_inl.at(j).z +
			t12.at<float>(2);
		float sigma_0 = 1.414 * sigma_px * depth_0 * depth_0 / (baseline * focal_kf0);

		float depth_1 = (float) points3D_align1_inl.at(j).z;
		float sigma_1 = 1.414 * sigma_px * depth_1 * depth_1 / (baseline * focal_kf1);

		z_offsets.push_back(std::fabs(depth_0 - depth_1));

		if (z_offsets.back() < (sigma_0 + sigma_1))
			counter_ok++;
	}

	bool accept = (float) counter_ok / (float) z_offsets.size() > perc ? true : false;
	if(!accept) {
		INFO("Visual match not validated (pre-opt). z_ok: " +
		  std::to_string((float) counter_ok / (float) z_offsets.size()) + " percent");
		return false;
	}

	// Post-match refinement

	// 1) Reproject keypoints from kf2 to kf1 and search for more matches
	std::vector<cv::DMatch> more_matches;
	std::vector<cv::Point3f> points3D_more = points3D_align0_inl;
	std::vector<cv::Point2f> features_more = keypoints_align1_inl;
	float rmse_augm;
	Eigen::Matrix4f T12_augm;
	if (searchMatches(points3D_unmatched0, desc_unmatched0,
					   keypoints_unmatched1, desc_unmatched1,
				   	   K0, K1, r12, t12, more_matches)) {

	    // Prepare inputs
		for (const auto& m:more_matches) {
			points3D_more.push_back(points3D_unmatched0.at(m.queryIdx));
			features_more.push_back(keypoints_unmatched1.at(m.trainIdx));
		}

		// Re-run p3p_ransac
		cv::Mat r12_ref, t12_ref;
		cv::solvePnPRansac(
			points3D_more,
			features_more,
			K1,
			cv::Mat(),
			r12,
			t12,
			false,
			m_options.pnp_max_iterations,
			m_options.pnp_max_epipolar_error,
			m_options.pnp_confidence,
			inl,
			cv::SOLVEPNP_P3P);

		cv::Mat R12_augm;
		cv::Rodrigues(r12, R12_augm);
		R12_augm.convertTo(R12_augm, CV_32FC1);
		t12.convertTo(t12, CV_32FC1);
		std::vector<float> repr_noise;
		computeReprojectionError(features_more, points3D_more, R12_augm, t12, K0,
								 repr_noise, rmse_augm);
		T12_augm = cvRt2eigen(R12_augm, t12);
        
        // If the original match is validated from fragile correspondences, the solution can change a lot
        // after searching for additional matches and re-running P3P_RANSAC. In this case, reject!
        if (rmse_augm > 3 * rmse) {
            std::stringstream ss; 
            ss << "in LCDetector::validateMatch(): rejected match as RMSE_augm (" << rmse_augm << ") > 3 * RMSE (" << rmse << ")";
            WARN(ss.str());
            return false;
        }
	};

	res.n_tot_matches = features_more.size();
	res.n_inliers = inl.rows;
	res.RMSE = rmse_augm;
	res.tf_q_db = T12_augm;
	res.dT = delta_t;
	res.dT_depths = delta_t_depths;
    res.cov_q_db = cov;

	if (visualize) {
		float res_factor = 1.0;
		bool side_by_side = true;

		cv::Mat I_query, I_database, display, display_rgb, display_rgb_res;
		std::vector<cv::Point2f> keypoints_query, keypoints_database;

		m_kmap.at(id0)->getImage(I_query);
		m_kmap.at(id1)->getImage(I_database);

		cv::Point2f hshift(0.0, 0.0);
		if (side_by_side) {
			cv::hconcat(I_query, I_database, display);
			hshift = cv::Point2f(I_query.cols, 0);
		} else {
			cv:addWeighted(I_query, .5, I_database, .5, 0, display);
		}
		cv::cvtColor(display, display_rgb, cv::COLOR_GRAY2BGR);
		cv::resize(display_rgb, display_rgb_res, cv::Size(), res_factor, res_factor);

		// Matches
		for (const auto& pt:keypoints_align0) {
			cv::circle(display_rgb_res, res_factor*pt, 1, cv::Scalar(0, 100, 255), 1, 8);
		}
		for (const auto& pt:keypoints_align1) {
			cv::circle(display_rgb_res, res_factor*pt + res_factor*hshift, 1, cv::Scalar(0, 100, 255), 1, 8);
		}

		// RANSAC inliers
		for (const auto& pt:keypoints_align0_inl) {
			cv::circle(display_rgb_res, res_factor*pt, 1, cv::Scalar(255, 0, 100), 1, 8);
		}
		for (const auto& pt:keypoints_align1_inl) {
			cv::circle(display_rgb_res, res_factor*pt + res_factor*hshift, 1, cv::Scalar(255, 0, 100), 1, 8);
		}
		for (size_t j=0; j<points3D_align0_inl.size(); j++) {
			cv::line(display_rgb_res,
					 res_factor*keypoints_align0_inl.at(j),
					 res_factor*keypoints_align1_inl.at(j) + res_factor*hshift,
					 cv::Scalar(0, 150, 0), 1, 8);
		}

		// RANSAC Outliers
		/*
		for (const auto& pt:keypoints_align0_out) {
			cv::circle(display_rgb_res, res_factor*pt, 1, cv::Scalar(0, 255, 100), 1, 8);
		}
		for (const auto& pt:keypoints_align1_out) {
			cv::circle(display_rgb_res, res_factor*pt + res_factor*hshift, 1, cv::Scalar(0, 255, 100), 1, 8);
		}
		for (size_t j=0; j<points3D_align0_out.size(); j++) {
			cv::line(display_rgb_res,
					 res_factor*keypoints_align0_out.at(j),
					 res_factor*keypoints_align1_out.at(j) + res_factor*hshift,
					 cv::Scalar(0, 0, 150), 1, 8);
		}*/

		// Visualize unmatched
		for (const auto& pt:keypoints_unmatched0) {
			cv::circle(display_rgb_res, res_factor*pt, 3, cv::Scalar(0, 100, 200), 1, 8);
		}
		for (const auto& pt:keypoints_unmatched1) {
			cv::circle(display_rgb_res, res_factor*pt + res_factor*hshift, 3, cv::Scalar(255, 0, 255), 1, 8);
		}
		for (const auto& m:more_matches) {
			cv::line(display_rgb_res,
					 res_factor*keypoints_unmatched0.at(m.queryIdx),
					 res_factor*keypoints_unmatched1.at(m.trainIdx) + res_factor*hshift,
					 cv::Scalar(0, 150, 150), 1, 8);
		}

		std::stringstream ss0, ss1, ss2, ss3, ss4, ss5, ss6, ss7;
		ss0 << "Loop validated: (" << id0 << ", " << id1 << ")";
		ss1 << "n_matches_initial: " << keypoints_align0.size();
		ss2 << "n_matches_inliers: " << keypoints_align0_inl.size();
		ss3 << "n_matches_depth_valid: " << counter_ok << " / " <<  z_offsets.size();
		ss4 << "n_matches_augmented: " << inl.size();
		ss5 << "RMSE: " << rmse;
		ss6 << "RMSE_augm: " << rmse_augm;
		ss7 << "dDepth\%: " << delta_t_depths;
		cv::putText(display_rgb_res, ss0.str(), cv::Point(20, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss1.str(), cv::Point(20, 35), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss2.str(), cv::Point(20, 50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss3.str(), cv::Point(20, 65), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss4.str(), cv::Point(20, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss5.str(), cv::Point(20, 95), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,255,0), 1, false);
		cv::putText(display_rgb_res, ss6.str(), cv::Point(20, 110), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255), 2, false);
		cv::putText(display_rgb_res, ss7.str(), cv::Point(20, 125), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255), 2, false);

        std::stringstream ss;
        ss << "/tmp/" << id0 << "_" << id1 << ".png";
        cv::imwrite(ss.str(), display_rgb_res);

		cv::imshow("Loop Validation Visualizer", display_rgb_res);
		cv::waitKey(20);
	}

	return accept;
};

template<int BITSIZE>
bool LCDetector<BITSIZE>::localizeFrame(const std::shared_ptr<LCD::Frame>& f, LCD::Result& result) {
	if (!f->isInit()) {
		WARN("in LCDetector::localizeFrame, frame is not initialized. Please fill with points");
		return false;
	}

	// lock!
	const std::lock_guard<std::mutex> lock(m_process_mutex);

	cv::Mat desc;
	std::vector<cv::Point2f> pts;
	f->getDescriptors(desc);
	f->getKeypoints(pts);

	const typename Tree::MatchableVector matchables(Tree::getMatchables(desc, pts));

	typename Tree::MatchVectorMap matches_per_image, matches_per_image_filt;
	p_tree->match(matchables, matches_per_image, m_tree_ratio * float(BITSIZE)); // .1 * bitsize

	// Check that n_matches is sufficient
	size_t image_id_max, n_matches;
	size_t max_matches = 0;

	for (const auto& m:matches_per_image) {
		const size_t database_id = m.first;
		if (m.second.size() >= max_matches) {
			max_matches = m.second.size();
			image_id_max = database_id;
		}
	}

	if (m_kmap.count(image_id_max) == 0) {
		std::stringstream ss;
		ss << "in LCDetector::localizeFrame, database keyframe id " << int(image_id_max) << " does not exist";
		ERROR(ss.str());
		return false;
	}

	// heuristic num. 1: fraction of matches
	size_t n_matchable_total = pts.size() + m_kmap.at(image_id_max)->size();
	float match_fraction = 2 * float(matches_per_image.at(image_id_max).size()) / float(n_matchable_total);
	bool is_enough_matches = match_fraction > m_match_ratio ? true : false;

	if (!is_enough_matches) {
		std::stringstream ss;
		ss << "in LCDetector::localizeFrame, match fraction not high enough " <<
		      "(" << match_fraction << ")";
		INFO(ss.str());
		return false;
	}

	// BF matching of all descriptors
	std::vector<cv::DMatch> matches, matches_filt;

	cv::Mat desc_db;
	size_t id_db = image_id_max;
	m_kmap.at(id_db)->getDescriptors(desc_db);

	std::shared_ptr<cv::BFMatcher> matcher_(new cv::BFMatcher(cv::NORM_HAMMING, true));
	matcher_->match(desc, desc_db, matches);

	// If zero matches return
	if(matches.size() < 1) {
		INFO("in LCDetector::localizeFrame, zero matches");
		return false;
	}

	std::vector<char> status;
	int good = 0;

	for(size_t m=0; m<matches.size(); m++) {
		if(matches.at(m).distance < 100) {
			status.push_back(1);
			matches_filt.push_back(matches.at(m));
			good++;
		} else {
			status.push_back(0);
		}
	}
	cv::Mat r12, t12;
	cv::Mat inl;

	std::vector<cv::Point3f> points3D_align;
	std::vector<cv::Point2f> keypoints_align;
	points3D_align.reserve(1000);
	keypoints_align.reserve(1000);

	// Select keypoints for PNP
	size_t count_have_depth = 0;
	for(size_t k=0; k<matches_filt.size(); k++) {

		// Check if there are zero depths
		if (m_kmap.at(id_db)->getPoint(matches_filt.at(k).trainIdx)->isInit() == 0 ||
			m_kmap.at(id_db)->getPoint(matches_filt.at(k).trainIdx)->z() > 20) {
			continue;
		}
		points3D_align.push_back(m_kmap.at(id_db)->getPoint(matches_filt.at(k).trainIdx)->point());
		keypoints_align.push_back(pts.at(matches_filt.at(k).queryIdx));
		count_have_depth++;
	}

	if (float(count_have_depth) / float(matches_filt.size()) < .5) {
		std::stringstream ss;
		ss << "in LCDetector::localizeFrame, too many points have no depth (" << int(count_have_depth) << " vs " <<
			matches_filt.size() << ")";
		INFO(ss.str());
		return false;
	}


	// P3P_RANSAC
	cv::solvePnPRansac(
		points3D_align,
		keypoints_align,
		m_kmap.at(id_db)->getCameraMatrix(),
		cv::Mat(),
		r12,
		t12,
		false,
		500,
		4.0,
		0.999,
		inl,
		cv::SOLVEPNP_P3P);

	if (float(inl.rows) / float(keypoints_align.size()) < .3) {
		std::stringstream ss;
		ss << "in LCDetector::localizeFrame, not enough RANSAC inliers: (" << int(inl.rows) << " vs " <<
			int(keypoints_align.size()) << ")";
		INFO(ss.str());
		return false;
	}

	// Decide and return
	cv::Mat R12_beforeLM;
	cv::Rodrigues(r12, R12_beforeLM);
	R12_beforeLM.convertTo(R12_beforeLM, CV_32FC1);
	t12.convertTo(t12, CV_32FC1);

	result.n_inliers = inl.rows;
	result.n_tot_matches = matches_filt.size();
	result.tf_q_db = cvRt2eigen(R12_beforeLM, t12); // such that p_cam = t_q_db * p_db
	result.database_id = id_db;

	return true;
};

void LCDetectorBase::printout(const std::string& message, const LCD::Verbosity& verbosity) {
	std::string prefix;
	switch (verbosity) {
		case LCD::Verbosity::INFO:
			prefix = "[ INFO] "; break;
		case LCD::Verbosity::WARN:
			prefix = "\033[1;32m[ WARN]\033[0m "; break;
		case LCD::Verbosity::ERROR:
			prefix = "\033[1;31m[ ERROR]\033[0m "; break;
		case LCD::Verbosity::ALWAYS:
			prefix = "\033[1;35m[ ALWAYS]\033[0m "; break;
	}
	if (verbosity >= verbosity_level)
		std::cout << prefix << message << std::endl;
};

// utilities
Eigen::Matrix4f cvRt2eigen(cv::Mat& R, cv::Mat& t) {

	if(R.cols != 3 && R.rows != 3 && t.rows != 3) {
		ERROR("in cvRt2eigen: R or t not [3x3] [3x1]");
	}

	// cv::Mats to float
	cv::Mat R_f, t_f;
	R.convertTo(R_f, CV_32FC1);
	t.convertTo(t_f, CV_32FC1);

	Eigen::Matrix4f T;

	T <<
	    R_f.at<float>(0, 0), R_f.at<float>(0, 1), R_f.at<float>(0, 2), t_f.at<float>(0),
		R_f.at<float>(1, 0), R_f.at<float>(1, 1), R_f.at<float>(1, 2), t_f.at<float>(1),
		R_f.at<float>(2, 0), R_f.at<float>(2, 1), R_f.at<float>(2, 2), t_f.at<float>(2),
		0                 , 0                 , 0                 , 1;

	return T;
};

cv::Mat eigen2mat(const Eigen::Matrix4f& input) {
	cv::Mat out(4,4,CV_32F);

	out.at<float>(0,0) = input(0,0);
	out.at<float>(0,1) = input(0,1);
	out.at<float>(0,2) = input(0,2);
	out.at<float>(0,3) = input(0,3);

	out.at<float>(1,0) = input(1,0);
	out.at<float>(1,1) = input(1,1);
	out.at<float>(1,2) = input(1,2);
	out.at<float>(1,3) = input(1,3);

	out.at<float>(2,0) = input(2,0);
	out.at<float>(2,1) = input(2,1);
	out.at<float>(2,2) = input(2,2);
	out.at<float>(2,3) = input(2,3);

	out.at<float>(3,0) = input(3,0);
	out.at<float>(3,1) = input(3,1);
	out.at<float>(3,2) = input(3,2);
	out.at<float>(3,3) = input(3,3);

	return out.clone();
};

void computeReprojectionError(const std::vector<cv::Point2f>& features, const std::vector<cv::Point3f>& points,
							  const cv::Mat& R, const cv::Mat& t, const cv::Mat& K, std::vector<float>& err, float& rmse) {
	err.clear();

	// Checks
	cv::Mat Rf, tf, Kf;
	R.convertTo(Rf, CV_32FC1);
	t.convertTo(tf, CV_32FC1);
	K.convertTo(Kf, CV_32FC1);

	std::vector<float> sq_errors;
	for (size_t i=0; i<features.size(); i++) {
		cv::Mat pt_loc = Rf * (cv::Mat_<float>(3, 1) << points.at(i).x, points.at(i).y, points.at(i).z) + tf;
		const float err_x = features.at(i).x - (Kf.at<float>(0, 0) * pt_loc.at<float>(0, 0) / pt_loc.at<float>(2, 0) + Kf.at<float>(0, 2));
		const float err_y = features.at(i).y - (Kf.at<float>(1, 1) * pt_loc.at<float>(1, 0) / pt_loc.at<float>(2, 0) + Kf.at<float>(1, 2));
		const float sq_err = err_x*err_x + err_y*err_y;
		sq_errors.push_back(sq_err);
		err.push_back(std::sqrt(sq_err));
	}

	float n = float(sq_errors.size());
	float sum = 0;
	for (const auto& e:sq_errors) {
		sum += e;
	}
	rmse = std::sqrt(sum / n);
}

/**
 * points_query 			vector of points from query keyframe, corresponding to non-matched features
 * desc_query				vector of descriptors, corresponding to points_query
 * features_db        		vector of features from database keyframe, corresponding to non-matched features
 * desc_db             		vector of descriptors from database keyframe, corresponding to features_db
 * K_query 					camera matrix from the query keyframe
 * K_db 					camera matrix from the database keyframe
 * rvec 					rodrigues vector aligning query to database
 * tvec 					translation vector aligning query to database
 * matches          		vector of matches found
**/
template<int BITSIZE>
bool LCDetector<BITSIZE>::searchMatches(
				   const std::vector<cv::Point3f>& points_query,
				   const cv::Mat& desc_query,
				   const std::vector<cv::Point2f>& features_db,
   				   const cv::Mat& desc_db,
			   	   const cv::Mat& K_query,
			       const cv::Mat& K_db,
			   	   const cv::Mat& rvec,
			   	   const cv::Mat& tvec,
			       std::vector<cv::DMatch>& matches) {

    matches.clear();

    // Checks
	if (points_query.empty() || features_db.empty() ||
		desc_query.empty() || desc_db.empty() ||
		K_query.empty() || K_db.empty() || rvec.empty() || tvec.empty()) {
		WARN("in searchMatches: some of the inputs are empty");
		return false;
	}

	if ( (points_query.size() != desc_query.rows) ||
	     (features_db.size() != desc_db.rows) ) {
 	 	WARN("in searchMatches: mismatched sizes of inputs");
		return false;
    }

    // project query points to database image
	std::vector<cv::Point2f> features_proj;
    cv::projectPoints(points_query, rvec, tvec, K_db, cv::Mat(), features_proj);

	if (features_proj.empty())
		return false;

	// Search for neighboring features in the database image
	for (size_t i=0; i<features_proj.size(); i++) {
		const cv::Point2f& f = features_proj.at(i);
		double min_hamming = std::numeric_limits<double>::max();
		int min_idx_db;
		for (size_t j=0; j<features_db.size(); j++) {
			if (std::fabs(f.x-features_db.at(j).x) < 2.0 &&
				std::fabs(f.y-features_db.at(j).y) < 2.0) {
				double hamming = cv::norm(desc_query.row(i), desc_db.row(j), cv::NORM_HAMMING);
				if (hamming < min_hamming) {
					min_hamming = hamming;
					min_idx_db = j;
				}
			}
		}

		if (min_hamming < .4 * BITSIZE) {
			/*std::cout << "Debug ----------\n" <<
			"hamming_dist: " << min_hamming << " between " <<
			"pt_i " << f.x << ", " << f.y << " -- pt_j: " << features_db.at(min_idx_db).x << ", " << features_db.at(min_idx_db).y << std::endl;*/
			matches.push_back(cv::DMatch(i, min_idx_db, min_hamming));
		}
	}

	if (matches.empty())
		return false;

	return true;
};
