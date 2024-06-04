#ifndef VOXEL_DILATER
#define VOXEL_DILATER(i, j, k, x, y, z, sy, sz, bx, by, bz, ck, ogm, ofst, val, fdl)                                                                                                                                                    \
(ck) = (x) == 0 || (x) == (bx) || (y) == 0 || (y) == (by) || (z) == 0 || (z) == (bz);                                                                                                                                                   \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                                   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0                     && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) - 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) >= 0    && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                                 && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x);     (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck)                && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0                    )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) - (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) >= 0      && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                                )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y);        (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx)                  && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) - (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) >= 0   )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z);        (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)                 )) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }  \
(i) = (x) + 1; (j) = (y) + (sy); (k) = (z) + (sz); (ofst) = (i) + (j) + (k); if ((!(ck) || ((ck) && (i) <= (bx) && (j) <= (by)   && (k) <= (bz))) && (ogm)[(ofst)] == 0) { (ogm)[(ofst)] = (val); (fdl).emplace_back((i), (j), (k)); }
#endif

// for the 3*3*3-1=26 voxels around the voxel (x, y, x);
// if if the current voxel is at the boundary of the grid using the conditions
// if not check, do; if check, check whether the voxel is in the point cloud bounds, and
// wether the point have been occupied, if not
// assign val (2) to the voxels map;
// 
// ogm : voxels, point map 
// ofst: idx the uid of the point in the point cloud
// val : value Dilated 1
// fdl : cevc, the surface voxel id