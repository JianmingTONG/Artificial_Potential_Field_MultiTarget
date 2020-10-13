#include<fstream>
#include<sstream>
#include<iostream>
#include<iomanip>
#include<string>
#include<opencv2/opencv.hpp>
#include<cstdlib>
#define HEIGHT 352
#define WIDTH  224
#define K_ATTRACT 1
#define ETA_REPLUSIVE 3
#define DIS_OBTSTACLE 3
#define INFLATION_RADIUS 5

int main(int argc, char** argv) {
    // ---------------------------------------- define variables;
    std::vector<std::vector<int>> obstacles, path, targets;
    std::vector<int> obstacle, target, currentLoc, goal;
    std::vector<float> infoGain;
    float currentPotential, tempInfoGain, minDis2Frontier;
    std::ifstream infile;
    cv::Mat mapmat;
    int map[HEIGHT][WIDTH];
    float dismap_backup[HEIGHT][WIDTH], dismap[HEIGHT][WIDTH], potentialmap[HEIGHT][WIDTH];


    // ---------------------------------------- initialize the potentialmap
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            potentialmap[i][j] = 0;
            dismap[i][j] = 0;
            dismap_backup[i][j] = 0;
        }
    }

    // ---------------------------------------- read map from external file & show image;
    infile.open("/home/jimmy/work/ROS_work/APF/apf_jianming_c/img_part.txt");
    mapmat = cv::imread("/home/jimmy/work/ROS_work/APF/apf_jianming_c/img_part.jpg", cv::IMREAD_UNCHANGED);
    cv::imshow("name1", mapmat);
    cv::waitKey(0);

    // ---------------------------------------- initialize the dismap & potential map from external files;
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            infile >> map[i][j];
//            if(map[i][j] == 100){
//                dismap_backup[i][j] = 0; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
//            }
//            else
            if(map[i][j] == -1){
                dismap_backup[i][j] = -1; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
            }
        }
    }
    infile.close();

    // ---------------------------------------- define the current point;
    currentLoc.push_back(73);
    currentLoc.push_back(114);
    path.push_back(currentLoc);

    // ---------------------------------------- visualize the potential map
    // visualize dismap;
    cv::Mat mapmat2;
    mapmat2 = cv::imread("/home/jimmy/work/ROS_work/APF/apf_jianming_c/img_part.jpg", cv::IMREAD_COLOR);
    std::cout << mapmat2.rows << " " << mapmat2.cols << std::endl;

    // ---------------------------------------- visualize path;
    int path_view[HEIGHT][WIDTH];
    for( int i = 0 ; i< HEIGHT; i++){
        for( int j = 0 ; j< HEIGHT; j++){
            path_view[i][j] = 0;
        }
    }

    // ------------------------------------------ find the obstacles & targets
    for (int i = 1; i < HEIGHT-1; i++){
        for (int j = 1; j < WIDTH-1; j++){
            if(map[i][j] == 100){
                obstacle.push_back(i);
                obstacle.push_back(j);
                obstacles.push_back(obstacle);
                std::vector<int>().swap(obstacle);
            }
            else if(map[i][j] == -1){
                // accessiable frontiers
                int numFree = 0, temp1 = 0;

                if (map[i + 1][j] == 0){
                    temp1 = temp1 + (map[i + 2][j    ] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j - 1] == 0) ? 1 : 0;
                    numFree += +(temp1 > 0);
                }

                if (map[i][j + 1] == 0){
                    temp1 = 0;
                    temp1 = temp1 + (map[i    ][j + 2] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 1][j + 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i - 1][j] == 0){
                    temp1 = 0;
                    temp1 = temp1 + (map[i - 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 1][j - 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 2][j    ] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i][j - 1] == 0){
                    temp1 = 0;
                    temp1 += (map[i    ][j - 2] == 0) ? 1 : 0;
                    temp1 += (map[i + 1][j - 1] == 0) ? 1 : 0;
                    temp1 += (map[i - 1][j - 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if( numFree > 0 ) {
                    target.push_back(i);
                    target.push_back(j);
                    targets.push_back(target);
                    std::vector<int>().swap(target);
                }
            }
        }
    }

    // ------------------------------------------ calculate infoGain of targets

    {
        // remove targets within the inflation radius of obstacles.
        std::cout << "number targets" << targets.size() << std::endl;
        int idx_target = 0;
        while (idx_target < targets.size()) {
            for (int i = 0; i < obstacles.size(); i++) {
                if (abs(targets[idx_target][0] - obstacles[i][0]) +
                    abs(targets[idx_target][1] - obstacles[i][1]) < INFLATION_RADIUS) {
                    targets.erase(targets.begin() + idx_target);
                    idx_target--;
                    break;
                }
            }
            idx_target++;
        }
        std::cout << "number targets after erase" << targets.size() << std::endl;
    }

    for(int  i = 0; i < targets.size(); i++){
//        if( ( map[targets[idx_target][0]+1][targets[idx_target][1]] == 100 ) || ( map[targets[idx_target][0]-1][targets[idx_target][1]] == 100 ) || ( map[targets[idx_target][0]][targets[idx_target][1]+1] == 100 )|| ( map[targets[idx_target][0]][targets[idx_target][1]-1] == 100 ) || ( map[targets[idx_target][0]+1][targets[idx_target][1]+1] == 100 ) || ( map[targets[idx_target][0]+1][targets[idx_target][1]-1] ==100 ) || ( map[targets[idx_target][0]-1][targets[idx_target][1]+1] == 100 ) || ( map[targets[idx_target][0]-1][targets[idx_target][1]-1] == 100 )){
////        if( ( map[targets[idx_target][0]+1][targets[idx_target][1]] == 100 ) || ( map[targets[idx_target][0]-1][targets[idx_target][1]] == 100 ) || ( map[targets[idx_target][0]][targets[idx_target][1]+1] == 100 )|| ( map[targets[idx_target][0]][targets[idx_target][1]-1] == 100 )){
//
//        }
        tempInfoGain = 0;
//        tempInfoGain += (map[targets[i][0]+1][targets[i][1]+1] == -1)?1:0;
//        tempInfoGain += (map[targets[i][0]+1][targets[i][1]-1] == -1)?1:0;
        tempInfoGain += (map[targets[i][0]+1][targets[i][1]  ] == -1)?1:0;
//        tempInfoGain += (map[targets[i][0]-1][targets[i][1]+1] == -1)?1:0;
//        tempInfoGain += (map[targets[i][0]-1][targets[i][1]-1] == -1)?1:0;
        tempInfoGain += (map[targets[i][0]-1][targets[i][1]  ] == -1)?1:0;
        tempInfoGain += (map[targets[i][0]  ][targets[i][1]+1] == -1)?1:0;
        tempInfoGain += (map[targets[i][0]  ][targets[i][1]-1] == -1)?1:0;

        infoGain.push_back(tempInfoGain);
    }

    // ------------------------------------------ set locations in dismap corresponding to targets as 0 to ensure that targets' distance will be calculated in the following operation. (dismapConstruction function).
    for(int i = 0; i< targets.size(); i++){
        dismap_backup[targets[i][0]][targets[i][1]] = 0;
    }

    for(int i = 0; i< obstacles.size(); i++){
        dismap_backup[obstacles[i][0]][obstacles[i][1]] = -2;
    }

    // ------------------------------------------ calculate path.
    int iteration = 1;

    currentPotential = 500;  // a random initialized value greater than all possible potentials.
    minDis2Frontier  = 500;  // a random initialized value greater than all possible distances.
    while(iteration < 2000 && minDis2Frontier > 1){

        // ------------------------------------------
        // ------------------------------------------
        // ------------------------------------------ get the minimial potential of the points around currentLoc
        {
            // ------------------------------------------ put locations around the current location into loc_around
            std::vector<double> potential;
            std::vector<int> curr_around;
            std::vector<std::vector<int>> loc_around;

            // upper
            curr_around.push_back(currentLoc[0]);
            curr_around.push_back(currentLoc[1]+1);
            loc_around.push_back(curr_around);

            // left
            std::vector<int>().swap(curr_around);
            curr_around.push_back(currentLoc[0]-1);
            curr_around.push_back(currentLoc[1]);
            loc_around.push_back(curr_around);

            // down
            std::vector<int>().swap(curr_around);
            curr_around.push_back(currentLoc[0]);
            curr_around.push_back(currentLoc[1]-1);
            loc_around.push_back(curr_around);

            // right
            std::vector<int>().swap(curr_around);
            curr_around.push_back(currentLoc[0]+1);
            curr_around.push_back(currentLoc[1]);
            loc_around.push_back(curr_around);


            // ------------------------------------------ calculate potentials of four neighbors of currentLoc
            for (int i = 0; i < loc_around.size(); i++){
                std::vector<int>().swap(curr_around);
                curr_around.push_back(loc_around[i][0]);
                curr_around.push_back(loc_around[i][1]);

                { // ------------------------------------------ calculate dismap
                    std::vector<std::vector<int> > curr_iter, next_iter;

                    // initialize the dis_map with number of dismapBackup
                    memcpy(dismap, dismap_backup,sizeof(float)*HEIGHT*WIDTH);

//                    for(int i = 0; i < obstacles.size(); i++){
//                        dismap[obstacles[i][0] ][obstacles[i][1] ] = -2;
//                    }

                    for(int i = 0; i < targets.size(); i++){
                        dismap[targets[i][0] ][targets[i][1] ] = 0;
                    }

                    int iter = 1;
                    curr_iter.push_back(curr_around);

                    // change pixel of the starting location to 10000 to avoid being changed. After processing dismap, we changed it back to 0;
                    dismap[curr_around[0]][curr_around[1]] = 10000;

                    while (curr_iter.size() > 0) {
                        for (int i = 0; i < curr_iter.size(); i++) {
                            if (dismap[curr_iter[i][0] + 1][curr_iter[i][1]] == 0) {
                                dismap[curr_iter[i][0] + 1][curr_iter[i][1]] = iter;
                                std::vector<int> tempLoc;
                                tempLoc.push_back(curr_iter[i][0] + 1);
                                tempLoc.push_back(curr_iter[i][1]);
                                next_iter.push_back(tempLoc);
                            }

                            if (dismap[curr_iter[i][0]][curr_iter[i][1] + 1] == 0) {
                                dismap[curr_iter[i][0]][curr_iter[i][1] + 1] = iter;
                                std::vector<int> tempLoc;
                                tempLoc.push_back(curr_iter[i][0]);
                                tempLoc.push_back(curr_iter[i][1] + 1);
                                next_iter.push_back(tempLoc);
                            }

                            if (dismap[curr_iter[i][0] - 1][curr_iter[i][1]] == 0) {
                                dismap[curr_iter[i][0] - 1][curr_iter[i][1]] = iter;
                                std::vector<int> tempLoc;
                                tempLoc.push_back(curr_iter[i][0] - 1);
                                tempLoc.push_back(curr_iter[i][1]);
                                next_iter.push_back(tempLoc);
                            }

                            if (dismap[curr_iter[i][0]][curr_iter[i][1] - 1] == 0) {
                                dismap[curr_iter[i][0]][curr_iter[i][1] - 1] = iter;
                                std::vector<int> tempLoc;
                                tempLoc.push_back(curr_iter[i][0]);
                                tempLoc.push_back(curr_iter[i][1] - 1);
                                next_iter.push_back(tempLoc);
                            }
                        }
                        curr_iter.swap(next_iter);
                        std::vector<std::vector<int> >().swap(next_iter);
                        iter++;
                    }

                    dismap[curr_around[0]][curr_around[1]] = 0.1;

                    // ----  reset invalid targets' distance value to 1000.
                    for (int i = 0; i < targets.size(); i++){
                        if(  (dismap[targets[i][0]][targets[i][1]] == 0) && ( (abs(targets[i][0] - curr_around[0]) + abs(targets[i][1] - curr_around[1])) > 1) ) {
                            dismap[targets[i][0]][targets[i][1]] = 1000;
                        }
                    }

                }

                { // ------------------------------------ calculate current potential
                    float attract = 0, repulsive = 0;
                    for (int i = 0; i < targets.size(); i++){
                        float temp = dismap[targets[i][0]][targets[i][1]];
                        if(temp < 0.1){
                            std::cout << "zero loc: (" <<  targets[i][0]   << ", " <<  targets[i][1] << ")" << " temp" << temp << std::endl;
                            std::cout << "curr loc: (" <<  curr_around[0]  << ", " << curr_around[1] << ")" << std::endl;
                        }
                        attract     = attract - K_ATTRACT*infoGain[i]/temp;
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
                    potential.push_back(attract + repulsive);
                }

            }

            std::vector<int>().swap(curr_around);

            // find the minimal potential around the current location
            std::vector<double>::iterator min_idx = std::min_element(potential.begin(), potential.end());

            path.push_back(loc_around[std::distance(std::begin(potential), min_idx)]);
            currentPotential = potential[std::distance(std::begin(potential), min_idx)];
        }

        path_view[path.back()[0]][path.back()[1]] = 1;
        currentLoc[0] = (path.back())[0];
        currentLoc[1] = (path.back())[1];

        {   // ------------------------------------------ calculate dismap
            std::vector<std::vector<int> > curr_iter, next_iter;

            // initialize the dis_map with number of dismapBackup
            memcpy(dismap, dismap_backup,sizeof(float)*HEIGHT*WIDTH);

//            for(int i = 0; i < obstacles.size(); i++){
//                dismap[obstacles[i][0] ][obstacles[i][1] ] = 0;
//            }

            for(int i = 0; i < targets.size(); i++){
                dismap[targets[i][0] ][targets[i][1] ] = 0;
            }

            int iter = 1;
            curr_iter.push_back(currentLoc);

            // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
            dismap[currentLoc[0]][currentLoc[1]] = -500;

            while (curr_iter.size() > 0) {
                for (int i = 0; i < curr_iter.size(); i++) {
                    if (dismap[curr_iter[i][0] + 1][curr_iter[i][1]] == 0) {
                        dismap[curr_iter[i][0] + 1][curr_iter[i][1]] = iter;
                        std::vector<int> tempLoc;
                        tempLoc.push_back(curr_iter[i][0] + 1);
                        tempLoc.push_back(curr_iter[i][1]);
                        next_iter.push_back(tempLoc);
                    }

                    if (dismap[curr_iter[i][0]][curr_iter[i][1] + 1] == 0) {
                        dismap[curr_iter[i][0]][curr_iter[i][1] + 1] = iter;
                        std::vector<int> tempLoc;
                        tempLoc.push_back(curr_iter[i][0]);
                        tempLoc.push_back(curr_iter[i][1] + 1);
                        next_iter.push_back(tempLoc);
                    }

                    if (dismap[curr_iter[i][0] - 1][curr_iter[i][1]] == 0) {
                        dismap[curr_iter[i][0] - 1][curr_iter[i][1]] = iter;
                        std::vector<int> tempLoc;
                        tempLoc.push_back(curr_iter[i][0] - 1);
                        tempLoc.push_back(curr_iter[i][1]);
                        next_iter.push_back(tempLoc);
                    }

                    if (dismap[curr_iter[i][0]][curr_iter[i][1] - 1] == 0) {
                        dismap[curr_iter[i][0]][curr_iter[i][1] - 1] = iter;
                        std::vector<int> tempLoc;
                        tempLoc.push_back(curr_iter[i][0]);
                        tempLoc.push_back(curr_iter[i][1] - 1);
                        next_iter.push_back(tempLoc);
                    }
                }
                curr_iter.swap(next_iter);
                std::vector<std::vector<int> >().swap(next_iter);
                iter++;
            }
            dismap[currentLoc[0]][currentLoc[1]] = 0.1;

            // ----  reset invalid targets' distance value to 1000.
            for (int i = 0; i < targets.size(); i++){
                if(  (dismap[targets[i][0]][targets[i][1]] == 0) && ( (abs(targets[i][0] - currentLoc[0]) + abs(targets[i][1] - currentLoc[1])) > 1) ) {
                    dismap[targets[i][0]][targets[i][1]] = 1000;
                }
            }

        }

        for (int i = 0; i < targets.size() ; i++){
            if(minDis2Frontier > dismap[targets[i][0]][targets[i][1]] ){
                minDis2Frontier = dismap[targets[i][0]][targets[i][1]];
            }
        }
        iteration++;
    }
    goal.push_back(path.back()[0]);
    goal.push_back(path.back()[1]);

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