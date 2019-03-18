#include "s2pcdf.h"

#include <netcdf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void s2pnc_init_netcdf_file(const char* dir_file,
   ncfile* ncf,
   int height,
   int length,
   int views,
   int row,
   int column)
  {
    char* file_name =  malloc(strlen(dir_file)+strlen(FILE_NAME)+1);
    strcpy(file_name,dir_file);
    strcat(file_name, FILE_NAME);
    ncf->name = file_name;

    int *ncid = &(ncf->ncid), *dimids = ncf->dimids, *varids = ncf->varids;

    int retval;

    if ((retval = nc_create(file_name, NC_CLOBBER, ncid)))
    ERROR(retval);

    if ((retval = nc_def_dim(*ncid, DLEN_NAME, length, &dimids[DLEN_IND])))
    ERROR(retval);
    if ((retval = nc_def_dim(*ncid, DHEI_NAME, height, &dimids[DHEI_IND])))
    ERROR(retval);
    if ((retval = nc_def_dim(*ncid, DVIE_NAME, views, &dimids[DVIE_IND])))
    ERROR(retval);

    if ((retval = nc_def_var(*ncid, VLON_NAME, NC_DOUBLE, 2, dimids, &varids[VLON_IND])))
    ERROR(retval);
    if ((retval = nc_def_var(*ncid, VLAT_NAME, NC_DOUBLE, 2, dimids, &varids[VLAT_IND])))
    ERROR(retval);
    if ((retval = nc_def_var(*ncid, VALT_NAME, NC_DOUBLE, 2, dimids, &varids[VALT_IND])))
    ERROR(retval);

    if ((retval = nc_def_var(*ncid, VVIE_NAME, NC_INT, NDIMS, dimids, &varids[VVIE_IND])))
    ERROR(retval);

    if ((retval = nc_def_var(*ncid, VSECHEI_NAME, NC_INT, NDIMS, dimids, &varids[VSECHEI_IND])))
    ERROR(retval);
    if ((retval = nc_def_var(*ncid, VSECLEN_NAME, NC_INT, NDIMS, dimids, &varids[VSECLEN_IND])))
    ERROR(retval);

    if ((retval = nc_def_var(*ncid, VLONSIGHT_NAME, NC_DOUBLE, NDIMS, dimids, &varids[VLONSIGHT_IND])))
    ERROR(retval);
    if ((retval = nc_def_var(*ncid, VLATSIGHT_NAME, NC_DOUBLE, NDIMS, dimids, &varids[VLATSIGHT_IND])))
    ERROR(retval);
    if ((retval = nc_def_var(*ncid, VALTSIGHT_NAME, NC_DOUBLE, NDIMS, dimids, &varids[VALTSIGHT_IND])))
    ERROR(retval);

    if ((retval = nc_put_att(*ncid, NC_GLOBAL, AROW_NAME, NC_INT, 1, &row)))
    ERROR(retval);
    if ((retval = nc_put_att(*ncid, NC_GLOBAL, ACOL_NAME, NC_INT, 1, &column)))
    ERROR(retval);

    if((retval = nc_enddef(*ncid)))
    ERROR(retval);
  }

void s2pnc_write_view(ncfile* ncf,
  int row,
  int column,
  int *viewId)
{
  int retval;
  int *ncid = &(ncf->ncid), *varids = ncf->varids, *dimids = ncf->dimids;
  size_t start[3] = {row, column, (*viewId)-1}, stop[3] = {1, 1, 1};

  if ((retval = nc_put_vara_int(*ncid, varids[VVIE_IND], start, stop, viewId)))
  ERROR(0);
}

void s2pnc_write_sigths_direction(ncfile* ncf,
  int row,
  int column,
  int viewId,
  double direction[3])
{
  int retval;
  int *ncid = &(ncf->ncid), *varids = ncf->varids, *dimids = ncf->dimids;
  size_t start[3] = {row, column, viewId-1}, stop[3] = {1, 1, 1};

  if ((retval = nc_put_vara_double(*ncid, varids[VLONSIGHT_IND], start, stop, &direction[0])))
  ERROR(0);
  if ((retval = nc_put_vara_double(*ncid, varids[VLATSIGHT_IND], start, stop, &direction[1])))
  ERROR(0);
  if ((retval = nc_put_vara_double(*ncid, varids[VALTSIGHT_IND], start, stop, &direction[2])))
  ERROR(0);
}

void s2pnc_write_slave_coordinate(ncfile* ncf,
  int row,
  int column,
  int viewId,
  double *imrow,
  double *imcolumn)
{
  int retval;
  int *ncid = &(ncf->ncid), *varids = ncf->varids, *dimids = ncf->dimids;
  size_t start[3] = {row, column, (viewId)-1}, stop[3] = {1, 1, 1};

  if ((retval = nc_put_vara_double(*ncid, varids[VSECHEI_IND], start, stop, imrow)))
    ERROR(0);
  if ((retval = nc_put_vara_double(*ncid, varids[VSECLEN_IND], start, stop, imcolumn)))
    ERROR(0);
}

void s2pnc_write_position(ncfile* ncf,
  int row,
  int column,
  double *lon,
  double *lat,
  double *alt)
{
  int retval;
  int *ncid = &(ncf->ncid), *varids = ncf->varids, *dimids = ncf->dimids;
  size_t start[2] = {row, column}, stop[2] = {1, 1};

  if ((retval = nc_put_vara_double(*ncid, varids[VLON_IND], start, stop, lon)))
    ERROR(0);
  if ((retval = nc_put_vara_double(*ncid, varids[VLAT_IND], start, stop, lat)))
    ERROR(0);
  if ((retval = nc_put_vara_double(*ncid, varids[VALT_IND], start, stop, alt)))
    ERROR(0);
}

void s2pnc_close_netcdf_file(ncfile* ncf)
{
  int retval, *ncid = &(ncf->ncid);
  free(ncf->name);

  if ((retval = nc_close(*ncid)))
    ERROR(retval);
}
