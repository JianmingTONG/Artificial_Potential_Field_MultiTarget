#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<opencv2/opencv.hpp>
#include<cstdlib>
#include<vector>
#include <math.h>


//#define HEIGHT 303
//#define WIDTH  175
#define HEIGHT 473
#define WIDTH  372
#define K_ATTRACT 1
#define ETA_REPLUSIVE 1
#define DIS_OBTSTACLE 4
#define INFLATION_RADIUS 5
#define IF_DEBUG


void dismapConstruction_start_target(int* dismap_, int* dismap_backup_, int* curr){
    std::vector<int *> curr_iter;
    std::vector<int *> next_iter;

    // initialize the dis_map with number of dismapBackup

    memcpy(dismap_, dismap_backup_, sizeof(int) * HEIGHT * WIDTH);

    int iter = 1;
    curr_iter.push_back(new int[2]{curr[0], curr[1]});

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dismap_[(curr[0]) * WIDTH + curr[1]] = -500;

    while (curr_iter.size() > 0) {
        for (int i = 0; i < curr_iter.size(); i++) {
            if (dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] + 1) * WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] + 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] - 1) * WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0]) * WIDTH + curr_iter[i][1] - 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0], curr_iter[i][1] - 1});
            }
        }
        curr_iter.swap(next_iter);
        std::vector<int *>().swap(next_iter);
        iter++;
    }
    dismap_[(curr[0]) * WIDTH + curr[1]] = 0;  // int only zero is available
    return ;
}

void dismapConstruction(int* dismap_, int* dismap_backup_, const std::vector<int* > targets, int* curr){
    std::vector<int* > curr_iter, next_iter;

    // initialize the dis_map with number of dismapBackup
    memcpy(dismap_, dismap_backup_,sizeof(int)*HEIGHT*WIDTH);

    for(int i = 0; i < targets.size(); i++){
        dismap_[(targets[i][0])*WIDTH + targets[i][1] ] = 0;
    }

    int iter = 1;
    curr_iter.push_back(curr);

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dismap_[(curr[0])*WIDTH + curr[1]] = -500;

    while (curr_iter.size() > 0) {
        for (int i = 0; i < curr_iter.size(); i++) {
            if (dismap_[(curr_iter[i][0] + 1)*WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] + 1)*WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] + 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0])*WIDTH + curr_iter[i][1] + 1] == 0) {
                dismap_[(curr_iter[i][0])*WIDTH + curr_iter[i][1] + 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0]    , curr_iter[i][1] + 1});
            }

            if (dismap_[(curr_iter[i][0] - 1)*WIDTH + curr_iter[i][1]] == 0) {
                dismap_[(curr_iter[i][0] - 1)*WIDTH + curr_iter[i][1]] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0] - 1, curr_iter[i][1]});
            }

            if (dismap_[(curr_iter[i][0])*WIDTH + curr_iter[i][1] - 1] == 0) {
                dismap_[(curr_iter[i][0])*WIDTH + curr_iter[i][1] - 1] = iter;
                next_iter.push_back(new int[2]{curr_iter[i][0]    , curr_iter[i][1] - 1});
            }
        }
        curr_iter.swap(next_iter);
        std::vector<int* >().swap(next_iter);
        iter++;
    }
    dismap_[(curr[0])*WIDTH + curr[1]] = 0.1;

    // ----  reset invalid targets' distance value to 1000.
    for (int i = 0; i < targets.size(); i++){
        if(  (dismap_[(targets[i][0])*WIDTH + targets[i][1]] == 0) && ( (abs(targets[i][0] - curr[0]) + abs(targets[i][1] - curr[1])) > 1) ) {
            dismap_[(targets[i][0])*WIDTH + targets[i][1]] = 1000;
        }
    }
    return ;
}


int main(int argc, char** argv) {
    // ---------------------------------------- define variables;
    std::vector<int* > obstacles, path, targets;
    int currentLoc[2], goal[2]; //target[2], obstacle[2]
    float  minDis2Frontier;
    std::ifstream infile;
    cv::Mat mapmat;
    int map[HEIGHT*WIDTH];
    std::vector<int * > dismap_targets_ptr;
    int* dismap_backup = new int[HEIGHT*WIDTH];

    // ---------------------------------------- initialize the potentialmap
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            dismap_backup[i*WIDTH + j] = 0;
        }
    }

    // ---------------------------------------- read map from external file & show image;
    infile.open("/home/jimmy/work/ROS_work/APF/apf_jianming_c/robot1map_out.txt");
    mapmat = cv::imread("/home/jimmy/work/ROS_work/APF/apf_jianming_c/img_part1.jpg", cv::IMREAD_UNCHANGED);
    cv::imshow("name1", mapmat);
    cv::waitKey(0);

    // ---------------------------------------- initialize the dismap & potential map from external files;
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            infile >> map[i*WIDTH + j];
            if(map[i*WIDTH + j] == -1){
                dismap_backup[i*WIDTH + j] = -1; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
            }
        }
    }
    infile.close();

    // ---------------------------------------- define the current point;
    currentLoc[0] = 89;
    currentLoc[1] = 161;
    path.push_back(new int[2]{89, 161});

    // ---------------------------------------- visualize the potential map
    // visualize dismap;
    cv::Mat mapmat2;
    mapmat2 = cv::imread("/home/jimmy/work/ROS_work/APF/apf_jianming_c/img_part1.jpg", cv::IMREAD_COLOR);
    std::cout << mapmat2.rows << " " << mapmat2.cols << std::endl;

    // ---------------------------------------- visualize path;
    int path_view[HEIGHT*WIDTH];
    for( int i = 0 ; i< HEIGHT; i++){
        for( int j = 0 ; j< WIDTH; j++){
            path_view[i*WIDTH + j] = 0;
        }
    }

    // ------------------------------------------ find the obstacles & targets
    for (int i = 2; i < HEIGHT-2; i++){
        for (int j = 2; j < WIDTH-2; j++){
            if(map[i*WIDTH + j] == 100){
                obstacles.push_back(new int[2]{i,j});
            }
            else if(map[i*WIDTH + j] == -1){
                // accessiable frontiers
                int numFree = 0, temp1 = 0;

                if (map[(i + 1)*WIDTH + j] == 0){
                    temp1 += (map[(i + 2)*WIDTH + j    ] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i*WIDTH + j + 1] == 0){
                    temp1 = 0;
                    temp1 += (map[      i*WIDTH + j + 2] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[(i - 1) *WIDTH + j] == 0){
                    temp1 = 0;
                    temp1 += (map[(i - 1)*WIDTH + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 2)*WIDTH + j    ] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i * WIDTH + j - 1] == 0){
                    temp1 = 0;
                    temp1 += (map[    i  *WIDTH + j - 2] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*WIDTH + j - 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if( numFree > 0 ) {
                    targets.push_back(new int[2]{i,j});
                }
            }
        }
    }
    std::cout << "number obstacles" << obstacles.size() << std::endl;

    {
        // remove targets within the inflation radius of obstacles.
        std::cout << "number targets" << targets.size() << std::endl;
        for(int idx_target = targets.size()-1; idx_target>=0; idx_target--) {
            for (int i = 0; i < obstacles.size(); i++) {
                if (abs(targets[idx_target][0] - obstacles[i][0]) +
                    abs(targets[idx_target][1] - obstacles[i][1]) < INFLATION_RADIUS) {
                    targets.erase(targets.begin() + idx_target);
                    break;
                }
            }
        }
        std::cout << "number targets after erase" << targets.size() << std::endl;
    }

    // ------------------------------------------ set locations in dismap corresponding to targets as 0 to ensure that targets' distance will be calculated in the following operation. (dismapConstruction function).
//    for(int i = 0; i< targets.size(); i++){
//        dismap_backup[(targets[i][0])*WIDTH + targets[i][1]] = 0;
//    }

    for(int i = 0; i< obstacles.size(); i++){
        dismap_backup[(obstacles[i][0])*WIDTH + obstacles[i][1]] = -2;
    }

    // ------------------------------------------ cluster targets into different groups and find the center of each group.
    // Note: x & y value of detected targets are in increasing order because of the detection is in laser scan order.
    std::vector<int* > target_process(targets);
    std::vector<int* > cluster_center;
    std::vector<int>   infoGain_cluster;

    while(target_process.size() > 0){
        std::vector<int* > target_cluster;
        target_cluster.push_back(target_process.back());
        target_process.pop_back();

        bool condition = true;
        while(condition){
            condition = false;
            int size_target_process = target_process.size();
            for (int i = size_target_process-1; i >= 0 ; i--){
                for (int j = 0; j < target_cluster.size(); j++){
                    int dis_ = abs(target_process[i][0] - target_cluster[j][0]) +  abs(target_process[i][1] - target_cluster[j][1]);
                    if(dis_ < 3){
                        target_cluster.push_back(target_process[i]);
                        target_process.erase(target_process.begin() + i);
                        condition = true;
                        break;
                    }
                }
            }
        }

        int center_[2]={0, 0};
        int num_ = target_cluster.size();
        for(int i = 0; i < num_; i++){
            center_[0] += target_cluster[i][0];
            center_[1] += target_cluster[i][1];
        }

        float center_float[2] = {float(center_[0]), float(center_[1])};
        center_float[0] = center_float[0]/float(num_);
        center_float[1] = center_float[1]/float(num_);

        float min_dis_ = 100.0;
        int min_idx_   = 10000;
        for(int i = 0; i < num_; i++){
            float temp_dis_ = abs(center_float[0]-float(target_cluster[i][0])) + abs(center_float[1]-float(target_cluster[i][1]));
            if(temp_dis_ < min_dis_){
                min_dis_ = temp_dis_;
                min_idx_ = i;
            }
        }

        cluster_center.push_back(new int[2]{target_cluster[min_idx_][0], target_cluster[min_idx_][1]});
        infoGain_cluster.push_back(num_);
    }

#ifdef IF_DEBUG
    for (int i = 0; i < cluster_center.size(); i++ ){
        std::cout << "number:" << infoGain_cluster[i] << " cluster center= (" << cluster_center[i][0] << " " <<  cluster_center[i][1] << std::endl;
    }
#endif

// ------------------------------------------ Generate Dismap starting from targets
//    int dismap_target[HEIGHT*WIDTH];
    int cluster_num = cluster_center.size();
    int** dismap_target = new int* [cluster_num];
    for (int i = 0; i<cluster_num; i++){
        dismap_target[i] = new int[HEIGHT*WIDTH];
    }
    for(int i_ = 0; i_ < cluster_num; i_++) {
        dismapConstruction_start_target(dismap_target[i_], dismap_backup, cluster_center[i_]);
        dismap_targets_ptr.push_back(dismap_target[i_]);
    }

    // ------------------------------------------ calculate path.
    int iteration = 1;
    minDis2Frontier  = 500;  // a random initialized value greater than all possible distances.
    while(iteration < 2000 && minDis2Frontier > 1){

        // ------------------------------------------
        // ------------------------------------------
        // ------------------------------------------ get the minimial potential of the points around currentLoc
        {
            // ------------------------------------------ put locations around the current location into loc_around
            float potential[4];
            int min_idx = 0;
            float min_potential = 10000;
            int* loc_around[4];

            // upper
            loc_around[0] = new int[2]{currentLoc[0], currentLoc[1]+1};
            // left
            loc_around[1] = new int[2]{currentLoc[0] - 1, currentLoc[1]};
            // down
            loc_around[2] = new int[2]{currentLoc[0]   , currentLoc[1] - 1};
            // right
            loc_around[3] = new int[2]{currentLoc[0] + 1, currentLoc[1]};

            // ------------------------------------------ calculate potentials of four neighbors of currentLoc
            for (int i = 0; i < 4; i++){
                int curr_around[2]={loc_around[i][0], loc_around[i][1]};

                { // ------------------------------------ calculate current potential
                    float attract = 0, repulsive = 0;
                    for (int i = 0; i < cluster_center.size(); i++){
                        int temp_int = dismap_targets_ptr[i][(curr_around[0])*WIDTH + curr_around[1]];
                        float temp = float(dismap_targets_ptr[i][(curr_around[0])*WIDTH + curr_around[1]]);
                        if(temp_int < 1){
                            std::cout << "zero loc: (" <<  cluster_center[i][0]   << ", " <<  cluster_center[i][1] << ")" << " temp" << temp << std::endl;
                            std::cout << "curr loc: (" <<  curr_around[0]  << ", " << curr_around[1] << ")" << std::endl;
                            continue;
                        }
                        attract     = attract - K_ATTRACT*infoGain_cluster[i]/temp;
                    }

                    for (int j = 0; j < obstacles.size(); j++){
                        float dis_obst = abs(obstacles[j][0]- curr_around[0]) + abs(obstacles[j][1]- curr_around[1]);
                        if( dis_obst <= DIS_OBTSTACLE) {
                            float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
                            repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
                        }
                    }

                    // to increase the potential if currend point has been passed before
                    for (int i =0; i < path.size(); i++){
                        if(curr_around[0] == path[i][0] && curr_around[1] == path[i][1]){
                            attract += 5;
                        }
                    }
                    potential[i] = attract + repulsive;
                    if(min_potential > potential[i] ){
                        min_potential = potential[i];
                        min_idx = i;
                    }
                }
            }
            std::cout << "potential" <<std::setprecision(5) << min_potential <<std::endl;
            path.push_back(loc_around[min_idx]);
        }

        path_view[(path.back()[0])*WIDTH + path.back()[1]] = 1;
        currentLoc[0] = (path.back())[0];
        currentLoc[1] = (path.back())[1];

        for (int i = 0; i < cluster_center.size() ; i++){
            int temp_dis_ =  dismap_targets_ptr[i][(currentLoc[0])*WIDTH + currentLoc[1]];
            if( (temp_dis_ == 0) && (abs(currentLoc[0]-cluster_center[i][0]) + abs(currentLoc[1]-cluster_center[i][1])) > 0){
                continue;
            }

            if(minDis2Frontier > temp_dis_ ){
                minDis2Frontier = temp_dis_;
            }
        }
        iteration++;
    }
    goal[0] = path.back()[0];
    goal[1] = path.back()[1];

    delete [] dismap_backup;
    for (int i = 0; i<cluster_num; i++){
        delete []  dismap_target[i];
    }
    delete [] dismap_target;

    // ------------------------------------------ draw obstacles in mapmat2.
    std::cout  << "number obstacles:" << obstacles.size() <<  std::endl;
    for(int i= 0; i< obstacles.size(); i++){
        cv::circle(mapmat2, cv::Point( (obstacles[i])[1], (obstacles[i])[0] ) , 1, cv::Scalar(0, 0, 255), 1);
    }

    // ------------------------------------------ draw path in mapmat2.
    std::cout  << "number path" << path.size() <<  std::endl;
    for(int i= 0; i< path.size(); i++){
        cv::circle(mapmat2, cv::Point( (path[i])[1], (path[i])[0]) , 1, cv::Scalar(0, 255, 0), 1);
    }

    // ------------------------------------------ visualiza mapmat2.
    cv::imshow("name", mapmat2);
    cv::waitKey(0);
    return 0;

}