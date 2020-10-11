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
#define ETA_REPLUSIVE 1
#define DIS_OBTSTACLE 5

void output_for_debug(float path_view[][WIDTH]){
    std::ofstream ofile_int;
    ofile_int.open("/home/nics/work/APF/apf_jianming_c/output_int_why_inf.txt");
    // output the dismap for debug
    for (int i = 0; i < HEIGHT; i++){
        for (int j = 0; j < WIDTH; j++){
//            ofile_int << std::setprecision(12) <<path_view[i][j] << " ";
            ofile_int << path_view[i][j] << " ";

        }
        ofile_int << std::endl;
    }
    ofile_int.close();
}

float computePotential(const float dis_map[][WIDTH], const std::vector<std::vector<int>> & obstacles, const std::vector<std::vector<int>> & targets, const std::vector<float> & infoGain, const std::vector<int> curr){
    float attract = 0, repulsive = 0;
    for (int i = 0; i < targets.size(); i++){
        float temp = dis_map[targets[i][0]][targets[i][1]];
        if(temp < 0.1){
            std::cout << "zero loc: (" <<  targets[i][0] << ", " <<  targets[i][1] << ")" << " temp" << temp << std::endl;
            std::cout << "curr loc: (" <<  curr[0]       << ", " <<        curr[1] << ")" << std::endl;
        }
        attract     = attract - K_ATTRACT*infoGain[i]/temp;
    }

    for (int j = 0; j < obstacles.size(); j++){
        float dis_obst = dis_map[ obstacles[j][0] ][ obstacles[j][1] ];
        if( dis_obst <= DIS_OBTSTACLE) {
            float temp = (1 / dis_obst - 1 / DIS_OBTSTACLE);
            repulsive = repulsive + 0.5 * ETA_REPLUSIVE * temp * temp;
        }
    }

    return attract + repulsive;
}

void dismapConstruction(const float dismapBackup[][WIDTH], float dis_map[][WIDTH], const std::vector<std::vector<int>> & obstacles, const std::vector<std::vector<int>> & targets, const std::vector<int> & curr) {
    std::vector<std::vector<int> > curr_iter, next_iter;

    // initialize the dis_map with number of dismapBackup
    memcpy(dis_map, dismapBackup,sizeof(float)*HEIGHT*WIDTH);

    for(int i = 0; i < obstacles.size(); i++){
        dis_map[obstacles[i][0] ][obstacles[i][1] ] = 0;
    }

    for(int i = 0; i < targets.size(); i++){
        dis_map[targets[i][0] ][targets[i][1] ] = 0;
    }

    int iter = 1;
    curr_iter.push_back(curr);

    // change pixel of the starting location to -500 to avoid being changed. After processing dismap, we changed it back to 0;
    dis_map[curr[0]][curr[1]] = -500;

    while (curr_iter.size() > 0) {
        for (int i = 0; i < curr_iter.size(); i++) {
            if (dis_map[curr_iter[i][0] + 1][curr_iter[i][1]] == 0) {
                dis_map[curr_iter[i][0] + 1][curr_iter[i][1]] = iter;
                std::vector<int> tempLoc;
                tempLoc.push_back(curr_iter[i][0] + 1);
                tempLoc.push_back(curr_iter[i][1]);
                next_iter.push_back(tempLoc);
            }


            if (dis_map[curr_iter[i][0]][curr_iter[i][1] + 1] == 0) {
                dis_map[curr_iter[i][0]][curr_iter[i][1] + 1] = iter;
                std::vector<int> tempLoc;
                tempLoc.push_back(curr_iter[i][0]);
                tempLoc.push_back(curr_iter[i][1] + 1);
                next_iter.push_back(tempLoc);
            }

            if (dis_map[curr_iter[i][0] - 1][curr_iter[i][1]] == 0) {
                dis_map[curr_iter[i][0] - 1][curr_iter[i][1]] = iter;
                std::vector<int> tempLoc;
                tempLoc.push_back(curr_iter[i][0] - 1);
                tempLoc.push_back(curr_iter[i][1]);
                next_iter.push_back(tempLoc);
            }

            if (dis_map[curr_iter[i][0]][curr_iter[i][1] - 1] == 0) {
                dis_map[curr_iter[i][0]][curr_iter[i][1] - 1] = iter;
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
    dis_map[curr[0]][curr[1]] = 0.1;
    return;
}

double computeMinAroundPotential(const float dismap_backup[][WIDTH], float dis_map[][WIDTH], const std::vector<std::vector<int>> & obstacles, const std::vector<std::vector<int>> & targets, const std::vector<float> & infoGain, const std::vector<int> & curr, std::vector<std::vector<int>>& nextLoc, const float currPotential ){
    std::vector<double> potential;
    std::vector<int> curr_around;
    std::vector<std::vector<int>> loc_around;

    // upper
    curr_around.push_back(curr[0]);
    curr_around.push_back(curr[1]+1);
    loc_around.push_back(curr_around);
    dismapConstruction(dismap_backup, dis_map, obstacles, targets, curr_around);
    potential.push_back(computePotential(dis_map, obstacles, targets, infoGain, curr_around));
    std::vector<int>().swap(curr_around);

    // left
    curr_around.push_back(curr[0]-1);
    curr_around.push_back(curr[1]);
    loc_around.push_back(curr_around);
    dismapConstruction(dismap_backup, dis_map, obstacles, targets, curr_around);
    potential.push_back(computePotential(dis_map, obstacles, targets, infoGain, curr_around));
    std::vector<int>().swap(curr_around);

    // down
    curr_around.push_back(curr[0]);
    curr_around.push_back(curr[1]-1);
    loc_around.push_back(curr_around);
    dismapConstruction(dismap_backup, dis_map, obstacles, targets, curr_around);
    potential.push_back(computePotential(dis_map, obstacles, targets, infoGain, curr_around));
    std::vector<int>().swap(curr_around);

    // right
    curr_around.push_back(curr[0]+1);
    curr_around.push_back(curr[1]);
    loc_around.push_back(curr_around);
    dismapConstruction(dismap_backup, dis_map, obstacles, targets, curr_around);
    potential.push_back(computePotential(dis_map, obstacles, targets, infoGain, curr_around));
    std::vector<int>().swap(curr_around);

    // find the minimal potential around the current location
    std::vector<double>::iterator min_idx = std::min_element(potential.begin(), potential.end());

    if(currPotential>potential[std::distance(std::begin(potential), min_idx)]){
        nextLoc.push_back(loc_around[std::distance(std::begin(potential), min_idx)]);
//        std::cout << "next location:" << (nextLoc.back())[0]  << " "  << (nextLoc.back())[1] << std::endl;
        return potential[std::distance(std::begin(potential), min_idx)];
    }
    else{
        // to avoid local minimum
        std::srand(time(NULL));
        int randIndex = std::rand()%4;
        nextLoc.push_back(loc_around[randIndex]);
//        std::cout << "next location:" << (nextLoc.back())[0]  << " "  << (nextLoc.back())[1] << std::endl;
        return potential[randIndex];
    }
}


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
        }
    }

    // ---------------------------------------- read map from external file & show image;
    infile.open("/home/nics/work/APF/apf_jianming_c/img_part.txt");
    mapmat = cv::imread("/home/nics/work/APF/apf_jianming_c/img_part.jpg", cv::IMREAD_UNCHANGED);
    cv::imshow("name1", mapmat);
    cv::waitKey(0);

    // ---------------------------------------- initialize the dismap & potential map from external files;
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            infile >> map[i][j];
            if(map[i][j] == 100){
                dismap_backup[i][j] = 0; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
            }
            else
            if(map[i][j] == 255){
                dismap_backup[i][j] = -400; // value of pixel ranged from [-128, 127] or [0, 255], this value should be the value outside of pixel's value range, and could be any other value within the range of integer.
            }
            else{
                dismap_backup[i][j] = 0;
            }
        }
    }
    infile.close();

    // ---------------------------------------- define the current point;
    currentLoc.push_back(239);
    currentLoc.push_back(129);
    path.push_back(currentLoc);

    // ---------------------------------------- visualize the potential map
    // visualize dismap;
    cv::Mat mapmat2;
    mapmat2 = cv::imread("/home/nics/work/APF/apf_jianming_c/img_part.jpg", cv::IMREAD_COLOR);
    std::cout << mapmat2.rows << " " << mapmat2.cols << std::endl;

    // ---------------------------------------- visualize path;
    int path_view[HEIGHT][WIDTH];
    for( int i = 0 ; i< HEIGHT; i++){
        for( int j = 0 ; j< HEIGHT; j++) {
            path_view[i][j] = 0;
        }
    }

    // ------------------------------------------ find the obstacles & targets
    for (int i = 1; i < HEIGHT-1; i++) {
        for (int j = 1; j < WIDTH-1; j++) {
            if(map[i][j] == 100){
                obstacle.push_back(i);
                obstacle.push_back(j);
                obstacles.push_back(obstacle);
                std::vector<int>().swap(obstacle);
            }
            else if(map[i][j] == 255) {
                // accessiable frontiers
                int numFree = 0, temp1 = 0;

                if (map[i + 1][j] == 0) {
                    temp1 = temp1 + (map[i + 2][j    ] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j - 1] == 0) ? 1 : 0;
                    numFree += +(temp1 > 0);
                }

                if (map[i][j + 1] == 0) {
                    temp1 = 0;
                    temp1 = temp1 + (map[i    ][j + 2] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i + 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 1][j + 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i - 1][j] == 0) {
                    temp1 = 0;
                    temp1 = temp1 + (map[i - 1][j + 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 1][j - 1] == 0) ? 1 : 0;
                    temp1 = temp1 + (map[i - 2][j    ] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i][j - 1] == 0) {
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
    for ( int i = 0; i < targets.size(); i++){
        tempInfoGain = 0;
        tempInfoGain += (map[targets[i][0]+1][targets[i][1]+1] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]+1][targets[i][1]-1] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]+1][targets[i][1]  ] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]-1][targets[i][1]+1] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]-1][targets[i][1]-1] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]-1][targets[i][1]  ] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]  ][targets[i][1]+1] == 255)?1:0;
        tempInfoGain += (map[targets[i][0]  ][targets[i][1]-1] == 255)?1:0;

        infoGain.push_back(tempInfoGain);
    }

    // ------------------------------------------ set locations in dismap corresponding to targets as 0 to ensure that targets' distance will be calculated in the following operation. (dismapConstruction function).
    for(int i = 0; i< targets.size(); i++){
        dismap_backup[targets[i][0]][targets[i][1]] = 0;
    }
    for(int i = 0; i< obstacles.size(); i++){
        dismap_backup[obstacles[i][0]][obstacles[i][1]] = 0;
    }

    // ------------------------------------------ print the obstacles and targets' size.
    std::cout << obstacles.size() << std::endl;
    std::cout << targets.size()   << std::endl;

    // for debug------------------------------------------ generate distance map named dismap.
//    dismapConstruction(dismap_backup, dismap, obstacles, targets, currentLoc);

    // for debug------------------------------------------ calculate potential map.
    // ------------------------------------------ might be redendunt
    for (int i = 0; i < HEIGHT; i++) {
        for (int j = 0; j < WIDTH; j++) {
            if(dismap_backup[i][j] >= 0){
                std::vector<int> temploc;
                temploc.push_back(i);
                temploc.push_back(j);
                dismapConstruction(dismap_backup, dismap, obstacles, targets, temploc);
                potentialmap[i][j] = computePotential(dismap, obstacles, targets, infoGain, temploc);
            }
        }
    }

    output_for_debug(potentialmap);

    // ------------------------------------------ calculate path.
    int iteration = 1;

    currentPotential = 500;  // a random initialized value greater than all possible potentials.
    minDis2Frontier  = 500;  // a random initialized value greater than all possible distances.
    while(iteration < 2000 && minDis2Frontier > 1){
        float minPotentialAround;
        // get the minimial potential of the points

        currentPotential = computeMinAroundPotential(dismap_backup, dismap, obstacles, targets, infoGain, currentLoc, path, currentPotential);
        path_view[path.back()[0]][path.back()[1]] = 1;
        currentLoc[0] = (path.back())[0];
        currentLoc[1] = (path.back())[1];
        dismapConstruction(dismap_backup, dismap, obstacles, targets, currentLoc);
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