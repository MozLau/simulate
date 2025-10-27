/* Kilobot Edge following demo
 *
 * Ivica Slavkov, Fredrik Jansson  2015
 */



#include <math.h>
#include<stdbool.h>
#include <kilombo.h>

#include "formation.h"
#include "shape.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



// Function to read two groups of hex points from a file
void read_boundary_from_file(const char *filename,struct Hex **group0, int *group0_size, struct Hex **group1, int *group1_size, struct Hex **group2, int *group2_size, int **recycleFlagSeq) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    char buffer[256];
    //Count lines in the first group
    int shape_count = 0;
    while (fgets(buffer, sizeof(buffer), file)) {
        if (strcmp(buffer, "\r\n") ==0 || strcmp(buffer, "\n") ==0 ) break; // Stop at the empty line
        //printf("%d", buffer[1]);
        shape_count++;
    }

    // Count lines in the second group    
    int first_count = 0;    
    while (fgets(buffer, sizeof(buffer), file)) {
        if (strcmp(buffer, "\r\n") ==0 || strcmp(buffer, "\n") ==0 ) break; // Stop at the empty line
        //printf("%d", buffer[1]);
        first_count++;
    }

    // Count lines in the third group
    int second_count = 0;
    while (fgets(buffer, sizeof(buffer), file)) {
        if (strcmp(buffer, "\r\n") ==0 || strcmp(buffer, "\n") ==0 ) break; // Stop at the empty line
        //printf("%d", buffer[1]);
        second_count++;
    }

    // Allocate memory for the first group
    *group0 = malloc(shape_count * sizeof(struct Hex));
    if (*group0 == NULL) {
        perror("Failed to allocate memory for group0");
        fclose(file);
        exit(EXIT_FAILURE);
    }
    *group0_size =shape_count;

    // Allocate memory for the s[]
    *group1 = malloc((first_count+second_count) * sizeof(struct Hex));
    if (*group1 == NULL) {
        perror("Failed to allocate memory for group1");
        fclose(file);
        exit(EXIT_FAILURE);
    }
    *group1_size =(first_count+second_count);

    // Allocate memory for the t[]
    *group2 = malloc((first_count+second_count) * sizeof(struct Hex));
    *recycleFlagSeq = malloc((first_count+second_count) * sizeof(int));
    if (*group2 == NULL) {
        perror("Failed to allocate memory for group2");
        fclose(file);
        free(*group1);
        exit(EXIT_FAILURE);
    }
    *group2_size = (first_count+second_count);

    // Rewind to read the first group
    rewind(file);
    for (int i = 0; i < shape_count; i++) {
        fscanf(file, "%d %d", &((*group0)[i].q), &((*group0)[i].r));
    }

    fgets(buffer, sizeof(buffer), file);
    fgets(buffer, sizeof(buffer), file);

    for (int i = 0; i < first_count; i++) {
        fscanf(file, "%d %d %d %d", &((*group1)[i].q), &((*group1)[i].r),&((*group2)[i].q), &((*group2)[i].r));
        (*recycleFlagSeq)[i] = 0;
    }

    // Skip the last newline and the following empty line
    fgets(buffer, sizeof(buffer), file);
    fgets(buffer, sizeof(buffer), file);

    // Read the third group
    
    for (int i = first_count; i < first_count+second_count; i++) {
        // fscanf(file, "%d %d", &((*group2)[i].q), &((*group2)[i].r));
        fscanf(file, "%d %d %d %d %d", &((*group1)[i].q), &((*group1)[i].r),&((*group2)[i].q), &((*group2)[i].r), &((*recycleFlagSeq)[i]));
    }

    //read number of bot
    fgets(buffer, sizeof(buffer), file);    
    fscanf(file, "%*d\n");
    // fscanf(file, "%*d\n", &mydata->num_bots_in_hole);

    
    //read shape offset
    fscanf(file, "%*d %*d\n");

    fclose(file);
}

void read_path_from_file(const char *filename, struct Hex **path, int **path_length, int* total_number_of_path) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    char buffer[1024];
    int section_count = 0;
    int line_count = 0;
    int *section_lines = NULL;
    int section_capacity = 0;

    int in_section = 0;

    while (fgets(buffer, sizeof(buffer), file)) {
        // // 移除行末的换行符
        // size_t len = strlen(buffer);
        // printf("%s: %d\n\n",buffer,len);
        // if (len > 0) {
        //     if (buffer[len-1] == '\n') buffer[len-1] = '\0';
        //     if (strlen(buffer) > 1 && buffer[len-2] == '\r') buffer[len-2] = '\0';
        // }
        // printf(" %d\n\n",len);

        // 判断是否为空行
        int is_empty = (strlen(buffer) == 2);

        if (is_empty) {
            if (in_section) {
                // 结束当前部分
                if (section_count >= section_capacity) {
                    section_capacity = (section_capacity == 0) ? 10 : section_capacity * 2;
                    section_lines = (int *)realloc(section_lines, section_capacity * sizeof(int));
                    if (!section_lines) {
                        perror("内存分配失败");
                        fclose(file);
                        return 1;
                    }
                }
                section_lines[section_count++] = line_count;
                in_section = 0;
                line_count = 0;
            }
            // 连续的空行被忽略
        } else {
            if (!in_section) {
                // 开始一个新部分
                in_section = 1;
                line_count ++;
            } else {
                // 继续当前部分
                line_count++;
            }
        }
    }

    // 处理文件末尾的部分
    if (in_section) {
        if (section_count >= section_capacity) {
            section_capacity = (section_capacity == 0) ? 10 : section_capacity * 2;
            section_lines = (int *)realloc(section_lines, section_capacity * sizeof(int));
            if (!section_lines) {
                perror("内存分配失败");
                fclose(file);
                return 1;
            }
        }
        section_lines[section_count++] = line_count;
    }

    

    // 输出统计结果
    printf("文件共有 %d 个部分\n", section_count);
    for (int i = 0; i < section_count; i++) {
        printf("第 %d 部分到 %d 行（不含空行）\n", i + 1, section_lines[i]);
    }

    //Allocate memory
    int total_lines = 0;
    for (int i = 0; i < section_count; i++) {
        total_lines += section_lines[i];
    }
    *path = malloc(total_lines * sizeof(struct Hex));
    
    *path_length = malloc(section_count * sizeof(int));

    // Rewind to read the first group
    rewind(file);
    int point_index = 0;
    for(int s = 0; s < section_count; s++)
    {
        for (int i = 0; i < section_lines[s]; i++) {
            // k = fscanf(file, "%d %d", &((*path)[point_index].q), &((*path)[point_index].r)); 
            fscanf(file, "%d %d", &((*path)[point_index].q), &((*path)[point_index].r)); 
            // printf("success %d\n ",k);
            // printf("Read: %d %d\n", (*path)[point_index].q, (*path)[point_index].r);
            point_index ++;
        }
        fgets(buffer, sizeof(buffer), file);

        (*path_length)[s] = section_lines[s];
    }

    *total_number_of_path = section_count;

    fclose(file);

    // free(section_lines);
    return 0;

}



void read_ribbon_head_from_file(const char *filename, struct RibbonHead **group1, int *group1_size) {
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Failed to open file");
        exit(EXIT_FAILURE);
    }

    // Count lines in the first group
    int first_count = 0;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), file)) {
        first_count++;
    }

    // Allocate memory for the first group
    *group1 = malloc(first_count * sizeof(struct RibbonHead));
    if (*group1 == NULL) {
        perror("Failed to allocate memory for group1");
        fclose(file);
        exit(EXIT_FAILURE);
    }
    *group1_size = first_count;

    // Rewind to read the first group
    rewind(file);
    for (int i = 0; i < first_count; i++) {
        fscanf(file, "%d %d %d %d", &((*group1)[i].q), &((*group1)[i].r), &((*group1)[i].t),&((*group1)[i].h));
    }

    fclose(file);
}









//outer polygen
#define NUM_OF_VERTEX 6
double vertex_x[NUM_OF_VERTEX+1] = {-200, -200,    500,   500,  1200,  1200, -200};
double vertex_y[NUM_OF_VERTEX+1] = {0,    -1400, -1400,  -700,  -700,     0,    0 };

bool in_poly(double x, double y)
{
    int num_of_intersection = 0;

    for(int i = 0; i <NUM_OF_VERTEX; i++)
    {
        if (fmin(vertex_y[i], vertex_y[i+1]) < y    &&     y < fmax(vertex_y[i], vertex_y[i+1])) //if the point is within two ys
        {
            if(x < fmax(vertex_x[i], vertex_x[i+1]))
            {
                double interection_x = (y - vertex_y[i])*(vertex_x[i+1] - vertex_x[i])/(vertex_y[i+1] - vertex_y[i]) + vertex_x[i];
            
                if(x < interection_x) num_of_intersection ++;
            }

        }
    }

    if(num_of_intersection%2 == 1) return true;
    else return false;
}

double dist_to_boundary(double x, double y)
{
    double min_dist = 1000000, dist, p1_x, p1_y, p2_x, p2_y;

    double p12, d, r;

    for(int i = 0; i <NUM_OF_VERTEX; i++)
    {
        //load the endpoints for the segment
        p1_x = vertex_x[i];
        p1_y = vertex_y[i];
        p2_x = vertex_x[i+1];
        p2_y = vertex_y[i+1];
        p12 = sqrt(pow(p1_x - p2_x, 2) +pow(p1_y - p2_y, 2)); //distance between p1 and p2

        //projection
        d = ((p2_x - p1_x) * (x - p1_x) + (p2_y - p1_y) * (y-p1_y))/p12;

        //ration of d, p12
        r= d/p12;
        if(r > 1) dist = sqrt( pow(x - p2_x,2) + pow(x-p2_y,2) );
        else if (r<0) dist = sqrt( pow(p1_x - x, 2)  + pow(p1_y-y,2) );
        else dist = sqrt( (pow(x-p1_x,2) + pow(y - p1_y,2) ) - d*d);

        //update min_dist
        if(dist < min_dist) min_dist = dist;
    }

    return min_dist;
}



bool in_rect(double rect_x, double rect_y, double rect_w, double rect_h, double x, double y) //return true if (x, y) is inside the rectangle
{
    if(x > rect_x && x < rect_x + rect_w && y > rect_y && y < rect_y + rect_h)
        return true;
    else
        return false;
}

// bool is_in_shape(double x, double y)
// {

//     if ( is_in_shape_total(x,y) && !is_in_shape_deleted(x,y) )
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }

// }





bool is_in_shape_total(double x, double y)
{

        if (in_rect(-100,-1500,1400,1400,x,y) && !in_rect(600,-1500,700,700,x,y)) 
        {
            return true;
        }
        else
        {
            return false;
        }

}

bool is_in_shape_deleted(double x, double y)
{

    return false;

}


//#define CUTOFF 90 //neighbors further away are ignored. (mm)

//radius is 17cm
//communication range is 70cm (edit in kilombo.json)

