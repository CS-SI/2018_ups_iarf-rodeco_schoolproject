#include "s2pcdf.h"

void s2pnc_init_netcdf_file(char* dir_file, ncfile* ncf,
   int width, int length, int views, int row, int col) {

  char* file_name =  malloc(strlen(dir_file)+strlen(FILE_NAME)+1);
  strcpy(file_name,dir_file);
  strcat(file_name, FILE_NAME);
  printf("%s",file_name);
  ncf->name = file_name;

  int *ncid = &(ncf->ncid), *dimids = ncf->dimids, *varids = ncf->varids;

  int retval;

  if ((retval = nc_create(file_name, NC_CLOBBER, ncid)))
    ERROR(retval);

  if ((retval = nc_def_dim(*ncid, DWID_NAME, width, &dimids[DWID_IND])))
    ERROR(retval);
  if ((retval = nc_def_dim(*ncid, DLEN_NAME, length, &dimids[DLEN_IND])))
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

  if ((retval = nc_put_att(*ncid, NC_GLOBAL, AROW_NAME, NC_INT, 1, &row)))
    ERROR(retval);
  if ((retval = nc_put_att(*ncid, NC_GLOBAL, ACOL_NAME, NC_INT, 1, &col)))
    ERROR(retval);

  if((retval = nc_enddef(*ncid)))
    ERROR(retval);

}

s2pnc_write_view_(ncfile* ncf, int viewId)
