#ifndef _S2P_CDF_H
#define _S2P_CDF_H

/* This is the name of the data file we will create. */
#define FILE_NAME "/data.nc"

/* Setting dimension of 2D grid of image width x length */
#define NDIMS 3
#define NVARS 6
#define NATRS 2
/* Dimensions are defined as DXXX_NAME for their name and DXXX_IND for the
/* indice position in the dimension table */
#define DWID_NAME "width"
#define DWID_IND 0
#define DHEI_NAME "height"
#define DHEI_IND 1
#define DVIE_NAME "sight"
#define DVIE_IND 2
/* variables are defined as VXXX_NAME for their name and VXXX_IND for the
/* indice position in the variable table */
#define VLON_NAME "longitude"
#define VLON_IND 0
#define VLAT_NAME "latitude"
#define VLAT_IND 1
#define VALT_NAME "altitude"
#define VALT_IND 2
#define VVIE_NAME "selected_sights"
#define VVIE_IND 3
#define VSECWID_NAME "width_pos_secondary_image"
#define VSECWID_IND 4
#define VSECHEI_NAME "height_pos_secondary_image"
#define VSECHEI_IND 5
/* attribute associated with the vile are define as AXXX_NAME for their name and VXXX_ind
/* for the indice in the attribute table */
#define AROW_NAME "width_pos_complete_reference"
#define AROW_IND 0
#define ACOL_NAME "height_pos_complete_reference"
#define ACOL_IND 1
/* Error handling */
#define ERROR(e) {printf("Error: %s\n", nc_strerror(e)); exit(2);}

typedef struct {
  char* name;
  int ncid;
  int dimids[NDIMS];
  int varids[NVARS];
} ncfile;

void s2pnc_init_netcdf_file(const char* dir_file,
   ncfile* ncf,
   int width,
   int length,
   int views,
   int row,
   int col);

void s2pnc_write_view(ncfile* ncf,
  int x,
  int y,
  int *viewId);

void s2pnc_write_slave_coordinate(ncfile* ncf,
  int x,
  int y,
  int viewId,
  double *imx,
  double *imy);

void s2pnc_write_position(ncfile* ncf,
  int x,
  int y,
  double *lon,
  double *lat,
  double *alt);

void s2pnc_close_netcdf_file(ncfile* ncf);

#endif
