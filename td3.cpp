#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* new_array= new double[new_size];
    for (int i=0; i<new_size ; i++){
        if (i<length){
            new_array[i]=array[i];
        }
        else{
            new_array[i]=0;
        }
    }
    delete[] array;
    return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double* n_array=new double[new_size];
    for (int i=0; i<new_size; i++) n_array[i]=array[i];
    delete[] array;
    return n_array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (current_size<max_size){
        array[current_size]=element;
        current_size++;
        return array;
    }
    max_size=max_size+5;
    double* nw_array=extend_array(array,current_size,max_size);
    nw_array[current_size]=element;
    current_size++;
    return nw_array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size--;
    array[current_size]=0;
    if (max_size-current_size==5){
        double* ne_array=shrink_array(array,max_size,current_size);
        max_size=max_size-5;
        return ne_array;
    }
    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;
  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      x = v0_x * t;
      y = v0_y * t  - 0.5 * g * t * t;
    }
  telemetry=append_to_array(t,telemetry,telemetry_current_size,telemetry_max_size);
  telemetry=append_to_array(x,telemetry,telemetry_current_size,telemetry_max_size);
  telemetry=append_to_array(y,telemetry,telemetry_current_size,telemetry_max_size);
  }
  return hit_target;
}

void sort(double *triplets, int &num_triplets) {
    double tmp_1;
    double tmp_2;
    double tmp_3;
    for (int i=0; i<(3*num_triplets);i=i+3){
        int pos=i;
        for (int j=i+3; j<(3*num_triplets);j=j+3){
            if (*(triplets+j)<*(triplets+pos)){
                pos=j;
            }
        }
    tmp_1=*(triplets+i);
    tmp_2=*(triplets+i+1);
    tmp_3=*(triplets+i+2);
    *(triplets+i)=*(triplets+pos);
    *(triplets+i+1)=*(triplets+pos+1);
    *(triplets+i+2)=*(triplets+pos+2);
    *(triplets+pos)=tmp_1;
    *(triplets+pos+1)=tmp_2;
    *(triplets+pos+2)=tmp_3;
    }
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_size,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
    global_telemetry_current_size=0;
    global_telemetry_max_size=0;
    for (int j=0; j<tot_telemetries; j++){
        for (int k=0; k<telemetries_size[j]; k++){
            global_telemetry=append_to_array(telemetries[j][k],global_telemetry,global_telemetry_current_size,global_telemetry_max_size);
        }
    }
    int num_triplets=global_telemetry_current_size/3;
    if (num_triplets > 0){
        sort(global_telemetry,num_triplets);
    }
}
