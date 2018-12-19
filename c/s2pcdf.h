#ifndef _S2P_CDF_H
#define _S2P_CDF_H

#include <netcdf.h>
#include <stdio.h>
#include <stdlib.h>

/* This is the name of the data file we will create. */
#define FILE_NAME "/data.nc"

/* Setting dimension of 2D grid of image width x length */
#define NDIMS 3
#define NVARS 4
#define NATRS 2
/* Dimensions are defined as DXXX_NAME for their name and DXXX_IND for the
/* indice position in the dimension table */
#define DWID_NAME "width"
#define DWID_IND 0
#define DLEN_NAME "length"
#define DLEN_IND 1
#define DVIE_NAME "views"
#define DVIE_IND 2
/* variables are defined as VXXX_NAME for their name and VXXX_IND for the
/* indice position in the variable table */
#define VLON_NAME "longitude"
#define VLON_IND 0
#define VLAT_NAME "latitude"
#define VLAT_IND 1
#define VALT_NAME "altitude"
#define VALT_IND 2
#define VVIE_NAME "views"
#define VVIE_IND 3
/* attribute associated with the vile are define as AXXX_NAME for their name and VXXX_ind
/* for the indice in the attribute table */
#define AROW_NAME "row"
#define AROW_IND 0
#define ACOL_NAME "column"
#define ACOL_IND 1

typedef struct {
  char* name;
  int ncid;
  int dimids[NDIMS];
  int varids[NVARS];
} ncfile;

void s2pnc_init_netcdf_file(char* dir_file, ncfile* ncf,
   int width, int length, int views, int row, int col);

void s2pnc_write_view(ncfile* ncf, int viewId);

void s2pnc_write_position(ncfile* ncf, int lon, int lat, int alt);

void s2pnc_close_netcdf_file(ncfile* ncf);

#endif
