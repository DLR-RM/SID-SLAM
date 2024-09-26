###################################################################################
IDNnav_path="/home/afontan/IDNav" 
executableFile="rgbdTUM"
dataset="Examples/RGB-D/TUM"

# Build IDNav
cd ${IDNnav_path}
rm -r build
mkdir build
cd build
	cmake ..
	make -j8
cd ..
		
sequenceNames=( "fr2_xyz" "fr2_rpy" "fr3_long_office" "fr2_desk" "fr1_xyz" "fr3_str_text_far" "fr3_str_text_near" "fr3_nostr_text_near")
sequenceNames=("fr3_str_text_far" "fr3_str_text_near" "fr3_nostr_text_near")
sequenceNames=("fr2_xyz" "fr2_rpy")
sequenceNames=("fr2_desk" "fr3_long_office")
sequenceNames=("fr1_xyz")
sequenceNames=("fr2_dishes" "fr1_plant")
sequenceNames=("fr3_str_text_far" "fr3_str_text_near" "fr3_nostr_text_near" "fr3_long_office" "fr2_xyz" "fr2_rpy")
sequenceNames=("fr1_plant")
for sequence in "${sequenceNames[@]}"
do
	cd ${IDNnav_path}
	sequencePath="${IDNnav_path}/${dataset}/${sequence}"
	###################################################################################
	# Prepare experiments
	expName="exp000"

	cd "resultsTUM"
		#rm -r ${sequence}
		#mkdir ${sequence}
		cd ${sequence}
			rm -r ${expName}
			mkdir ${expName}
			cd ${expName}
				expPath="${IDNnav_path}/resultsTUM/${sequence}/${expName}"
				
				rm "ate.txt"
				> "ate.txt"
				ATE_path="${expPath}/ate.txt"
				
				rm "ateFinal.txt"
				> "ateFinal.txt"
				ATEfinal_path="${expPath}/ateFinal.txt"
				
				rm "ateKey_raw.txt"
				> "ateKey_raw.txt"
				ATEkey_raw_path="${expPath}/ateKey_raw.txt"
				
				rm "ateKey_model.txt"
				> "ateKey_model.txt"
				ATEkey_model_path="${expPath}/ateKey_model.txt"

				rm "ateOrder.txt"
				> "ateOrder.txt"
				ATEOrder_path="${expPath}/ateOrder.txt"
				
				chmod +777 ${ATE_path}
				chmod +777 ${ATEfinal_path}
				chmod +777 ${ATEkey_raw_path}
				chmod +777 ${ATEkey_model_path}
			cd ..	
			
		cd ..
			
	cd ..
	###################################################################################
	# Execution loop
	cd "${dataset}"

	for i in {0..0}
	do
		./${executableFile} ../../../systemSettings.yaml ${sequence}/sequenceSettings.yaml ${i} ${expPath}
		echo "finish"
	done


	###################################################################################
	# Evaluation loop 
	# Paths
	evaluationTools_path="/home/afontan/datasetsTools/RGBD_TUM/"
	gt_path="/home/afontan/RGBD_TUM_datasets_temp"

	chmod 777 ${evaluationTools_path}evaluate_ate.py
	chmod 777 ${evaluationTools_path}evaluate_rpe.py
	chmod 777 ${evaluationTools_path}evaluate_ate_scale.py

	groundtruth_file="${gt_path}/${sequence}/groundtruth.txt"

		    
	#array=($(ls ${expPath}/*_traj_TUMformat_log.txt))	    	    
	#for j in "${array[@]}"
	#do	

	#	ate=$(python2 ${evaluationTools_path}evaluate_ate.py $groundtruth_file $j)
#		(echo $ate) >> $ATE_path
#		echo $ate
#	done
	
#	array=($(ls ${expPath}/*finalTrajectory_TUMformat_Log.txt))	    	    
#	for j in "${array[@]}"
#	do	
#		ate=$(python2 ${evaluationTools_path}evaluate_ate.py $groundtruth_file $j)
#		(echo $ate) >> $ATEfinal_path
#		echo $ate
#	done
	
	array=($(ls ${expPath}/*raw_keyframes_TUMformat_Log.txt))	    	    
	for j in "${array[@]}"
	do	
		ate=$(python2 ${evaluationTools_path}evaluate_ate.py $groundtruth_file $j)
		(echo $ate) >> $ATEkey_raw_path
		(echo $j) >> $ATEOrder_path
		echo $ate
	done
	for iModel in {1..10}
	do
		array=($(ls ${expPath}/*model${iModel}_keyframes_TUMformat_Log.txt))	    	    
		for j in "${array[@]}"
		do	
			ate=$(python2 ${evaluationTools_path}evaluate_ate.py $groundtruth_file $j)
			(echo $ate) >> $ATEkey_model_path
			(echo $j) >> $ATEOrder_path
			echo $ate
		done
	done
done
