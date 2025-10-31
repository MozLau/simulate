/* Kilobot Edge following demo
 *
 * Ivica Slavkov, Fredrik Jansson  2015
 */

void read_boundary_from_file(const char *filename, struct Hex **group0, int *group0_size, struct Hex **group1, int *group1_size, struct Hex **group2, int *group2_size, int **recycleFlagSeq) ;
bool is_in_shape_total(double x, double y);
bool is_in_shape_deleted(double x, double y);
//bool is_in_shape(double x, double y);
bool is_in_buffer(double x, double y);
bool in_poly(double x, double y);
double dist_to_boundary(double x, double y);