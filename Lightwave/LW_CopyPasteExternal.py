#! /usr/bin/env python
# -*- Mode: Python -*-
# -*- coding: ascii -*-

__author__     = "Oliver Hotz"
__date__       = "April 27, 2017"
__version__    = "1.0"
__maintainer__ = "Oliver Hotz"
__email__      = "oliver@origamidigital.com"
__status__     = "Copies / Pastes Objects between various 3d applications"
__lwver__      = "2015"

try:
    import lwsdk, os, tempfile, sys, time
except ImportError:
    raise Exception("The LightWave Python module could not be loaded.")


##########################################################################
#  Copy current mesh to temporary file for data exchange                 #
##########################################################################

class OD_LWCopyToExternal(lwsdk.ICommandSequence):
    def __init__(self, context):
        super(OD_LWCopyToExternal, self).__init__()
        self.pidx = 0
        self.poidx = 0
        # Use STABLE integer IDs as keys (not SWIG string reprs)
        self.pointidxmap = {}  # int(LWPntID) -> point index
        self.polyidxmap  = {}  # int(LWPolID) -> poly index

    def fast_point_scan(self, point_list, point_id):
        """Callback for fastPointScan: build list of points and point index map."""
        pid = int(point_id)
        point_list.append(point_id)
        self.pointidxmap[pid] = self.pidx
        self.pidx += 1
        return lwsdk.EDERR_NONE

    def fast_poly_scan(self, poly_list, poly_id):
        """Callback for fastPolyScan: build list of polys and poly index map."""
        pid = int(poly_id)
        poly_list.append(poly_id)
        self.polyidxmap[pid] = self.poidx
        self.poidx += 1
        return lwsdk.EDERR_NONE

    # LWCommandSequence -----------------------------------
    def process(self, mod_command):

        def polytree(polys, points):
            """
            Build a structure mapping each point to the list of polys using it
            and the sum of their normals.

            Returns:
                n           : list of lists of poly indices per point
                nfullNormals: accumulated poly normals per point
            """
            n = []
            nfullNormals = []

            # create empty arrays per point
            for _p in points:
                n.append([])
                nfullNormals.append([])

            count = 0
            for poly in polys:
                pts = mesh_edit_op.polyPoints(mesh_edit_op.state, poly)
                for p in pts:
                    pid = int(p)
                    pidx = self.pointidxmap[pid]

                    n[pidx].append(count)

                    # accumulate normals
                    if nfullNormals[pidx] == []:
                        nfullNormals[pidx] = lwsdk.Vector(
                            mesh_edit_op.polyNormal(mesh_edit_op.state, polys[count])[1]
                        )
                    else:
                        nfullNormals[pidx] = (
                            lwsdk.Vector(nfullNormals[pidx]) +
                            lwsdk.Vector(mesh_edit_op.polyNormal(mesh_edit_op.state, polys[count])[1])
                        )
                count += 1

            return n, nfullNormals

        # ----------------------------------------------------------------------
        # Deselect any Morph Targets
        # ----------------------------------------------------------------------
        command = mod_command.lookup(mod_command.data, "SELECTVMAP")
        cs_options = lwsdk.marshall_dynavalues(("MORF",))
        result = mod_command.execute(
            mod_command.data, command, cs_options, lwsdk.OPSEL_USER
        )

        # Temporary file used for data exchange
        file = tempfile.gettempdir() + os.sep + "ODVertexData.txt"

        # ----------------------------------------------------------------------
        # Find existing VMaps
        # ----------------------------------------------------------------------
        loaded_weight = []
        loaded_uv     = []
        loaded_morph  = []

        obj_funcs = lwsdk.LWObjectFuncs()

        for u in range(0, obj_funcs.numVMaps(lwsdk.LWVMAP_WGHT)):
            loaded_weight.append(obj_funcs.vmapName(lwsdk.LWVMAP_WGHT, u))

        for u in range(0, obj_funcs.numVMaps(lwsdk.LWVMAP_TXUV)):
            loaded_uv.append(obj_funcs.vmapName(lwsdk.LWVMAP_TXUV, u))

        for u in range(0, obj_funcs.numVMaps(lwsdk.LWVMAP_MORF)):
            loaded_morph.append(obj_funcs.vmapName(lwsdk.LWVMAP_MORF, u))

        # ----------------------------------------------------------------------
        # Start mesh edit operations
        # ----------------------------------------------------------------------
        edit_op_result = lwsdk.EDERR_NONE
        mesh_edit_op = mod_command.editBegin(0, 0, lwsdk.OPLYR_FG)
        if not mesh_edit_op:
            print >> sys.stderr, 'Failed to engage mesh edit operations!'
            return lwsdk.AFUNC_OK

        f = None

        try:
            # ------------------------------------------------------------------
            # Query all points
            # ------------------------------------------------------------------
            points = []
            edit_op_result = mesh_edit_op.fastPointScan(
                mesh_edit_op.state,
                self.fast_point_scan,
                (points,),
                lwsdk.OPLYR_FG,
                0
            )
            if edit_op_result != lwsdk.EDERR_NONE:
                mesh_edit_op.done(mesh_edit_op.state, edit_op_result, 0)
                return lwsdk.AFUNC_OK

            point_count = len(points)
            edit_op_result = lwsdk.EDERR_NONE

            # ------------------------------------------------------------------
            # Query all polygons
            # ------------------------------------------------------------------
            polys = []
            edit_op_result = mesh_edit_op.fastPolyScan(
                mesh_edit_op.state,
                self.fast_poly_scan,
                (polys,),
                lwsdk.OPLYR_FG,
                0
            )
            if edit_op_result != lwsdk.EDERR_NONE:
                mesh_edit_op.done(mesh_edit_op.state, edit_op_result, 0)
                return lwsdk.AFUNC_OK

            poly_count = len(polys)
            edit_op_result = lwsdk.EDERR_NONE

            # If there are no points, nothing to do
            if point_count == 0:
                lwsdk.LWMessageFuncs().info("No Points.", "")
                return lwsdk.AFUNC_OK

            # ------------------------------------------------------------------
            # Initialise containers
            # ------------------------------------------------------------------
            uvMaps        = []
            weightMaps    = []
            morphMaps     = []
            vertexNormals = []

            # ------------------------------------------------------------------
            # Write VERTICES section
            # ------------------------------------------------------------------
            f = open(file, "w")
            f.write("VERTICES:" + str(point_count) + "\n")

            # Point positions
            for point in points:
                pos = mesh_edit_op.pointPos(mesh_edit_op.state, point)
                # Note: Z is flipped as in original script
                f.write(
                    str(pos[0]) + " " +
                    str(pos[1]) + " " +
                    str(pos[2] * -1) + "\n"
                )

            # ------------------------------------------------------------------
            # Check if any surfaces have smoothing enabled
            # ------------------------------------------------------------------
            surf_funcs   = lwsdk.LWSurfaceFuncs()
            state_query  = lwsdk.LWStateQueryFuncs()
            surfIDs      = surf_funcs.byObject(state_query.object())
            global_smoothing = 0

            if surfIDs:
                for surf in surfIDs:
                    smooth_val = surf_funcs.getFlt(surf, lwsdk.SURF_SMAN)
                    if smooth_val > 0:
                        global_smoothing = 1
                        break

            # ------------------------------------------------------------------
            # Build poly tree only if any smoothing is enabled
            # ------------------------------------------------------------------
            ptree = None
            if global_smoothing > 0:
                ptree = polytree(polys, points)

            # ------------------------------------------------------------------
            # Write POLYGONS section
            # ------------------------------------------------------------------
            f.write("POLYGONS:" + str(len(polys)) + "\n")

            for poly in polys:
                # Surface name for this poly
                surfname = mesh_edit_op.polySurface(mesh_edit_op.state, poly)
                surfID   = surf_funcs.byName(surfname, state_query.object())
                surf_smoothing = 0.0
                if surfID and surfID[0] is not None:
                    surf_smoothing = surf_funcs.getFlt(surfID[0], lwsdk.SURF_SMAN)

                ppoint = ""

                # Reverse order as in original script
                for point in reversed(mesh_edit_op.polyPoints(mesh_edit_op.state, poly)):
                    pid  = int(point)
                    idx  = self.pointidxmap[pid]
                    ppoint += "," + str(idx)

                    # Vertex normals
                    if surf_smoothing > 0 and ptree is not None:
                        # Average of accumulated normals for this point
                        vertexNormals.append(
                            lwsdk.Vector().normalize(
                                ptree[1][idx] / float(len(ptree[0]))
                            )
                        )
                    else:
                        vertexNormals.append(
                            mesh_edit_op.polyNormal(mesh_edit_op.state, poly)[1]
                        )

                # Determine polygon type
                polytype = "FACE"
                subD = mesh_edit_op.polyType(mesh_edit_op.state, poly)
                if subD == lwsdk.LWPOLTYPE_SUBD:
                    polytype = "CCSS"
                elif subD == lwsdk.LWPOLTYPE_PTCH:
                    polytype = "SUBD"

                f.write(ppoint[1:] + ";;" + surfname + ";;" + polytype + "\n")

            # ------------------------------------------------------------------
            # Grab all UVs
            # ------------------------------------------------------------------
            for uvs in loaded_uv:
                cont    = []
                discont = []
                c       = 0

                # Select UV map
                mesh_edit_op.vMapSelect(
                    mesh_edit_op.state,
                    uvs,
                    lwsdk.LWVMAP_TXUV,
                    2
                )

                # Determine continuous vs discontinuous UVs
                for poly in polys:
                    for point in mesh_edit_op.polyPoints(mesh_edit_op.state, poly):
                        pInfo = mesh_edit_op.pointVPGet(
                            mesh_edit_op.state, point, poly
                        )[1]

                        pid  = int(point)
                        pidx = self.pointidxmap[pid]
                        polyidx = self.polyidxmap[int(poly)]

                        if pInfo is not None:
                            # Discontinuous UVs (per poly)
                            curPos = [pInfo[0], pInfo[1]]
                            discont.append([curPos, polyidx, pidx])
                            c += 1
                        else:
                            # Continuous UVs (per point)
                            v = mesh_edit_op.pointVGet(
                                mesh_edit_op.state, point
                            )[1]
                            if v is not None:
                                curPos = [v[0], v[1]]
                                cont.append([curPos, pidx])
                                c += 1

                # Write UVs section
                f.write("UV:" + uvs + ":" + str(c) + "\n")

                # Discontinuous entries
                for uvpos in discont:
                    # uvpos = [pos, polyidx, pidx]
                    f.write(
                        str(uvpos[0][0]) + " " + str(uvpos[0][1]) +
                        ":PLY:" + str(uvpos[1]) +
                        ":PNT:" + str(uvpos[2]) + "\n"
                    )

                # Continuous entries
                for uvpos in cont:
                    f.write(
                        str(uvpos[0][0]) + " " + str(uvpos[0][1]) +
                        ":PNT:" + str(uvpos[1]) + "\n"
                    )

            # ------------------------------------------------------------------
            # Grab all Morphs
            # ------------------------------------------------------------------
            for morph in loaded_morph:
                mesh_edit_op.vMapSelect(
                    mesh_edit_op.state,
                    morph,
                    lwsdk.LWVMAP_MORF,
                    3
                )
                f.write("MORPH:" + morph + "\n")

                for point in points:
                    val = mesh_edit_op.pointVGet(
                        mesh_edit_op.state,
                        point
                    )[1]
                    if val is not None:
                        f.write(
                            str(val[0]) + " " +
                            str(val[1]) + " " +
                            str(val[2] * -1) + "\n"
                        )
                    else:
                        f.write("0 0 0\n")

            # ------------------------------------------------------------------
            # Write VERTEXNORMALS section
            # ------------------------------------------------------------------
            f.write("VERTEXNORMALS:" + str(len(vertexNormals)) + "\n")
            for normal in vertexNormals:
                f.write(
                    str(normal[0]) + " " +
                    str(normal[1]) + " " +
                    str(normal[2] * -1) + "\n"
                )

        except:
            edit_op_result = lwsdk.EDERR_USERABORT
            raise
        finally:
            mesh_edit_op.done(mesh_edit_op.state, edit_op_result, 0)
            if f is not None:
                try:
                    f.close()
                except:
                    pass

        return lwsdk.AFUNC_OK


##########################################################################
#  Paste temporary data exchange file to current layer                   #
##########################################################################

class OD_LWPasteFromExternal(lwsdk.ICommandSequence):
    def __init__(self, context):
        super(OD_LWPasteFromExternal, self).__init__()

    # LWCommandSequence -----------------------------------
    def process(self, mod_command):
        # get the command arguments (so that we can also run this from layout)
        cmd = mod_command.argument.replace('"', '')

        file = tempfile.gettempdir() + os.sep + "ODVertexData.txt"

        # open the temp file
        if os.path.exists(file):
            with open(file, "r") as f:
                lines = f.readlines()
        else:
            lwsdk.LWMessageFuncs().info(
                "Storage File does not exist.  Needs to be created via the Layout CopyTransform counterpart",
                ""
            )
            return 0

        # if we are in Modeler (no argument), clear polys
        if cmd == "":
            command = mod_command.lookup(mod_command.data, "CUT")
            result = mod_command.execute(
                mod_command.data,
                command,
                None,
                lwsdk.OPLYR_FG
            )

        edit_op_result = lwsdk.EDERR_NONE
        mesh_edit_op = mod_command.editBegin(0, 0, lwsdk.OPSEL_USER)
        if not mesh_edit_op:
            print >> sys.stderr, 'Failed to engage mesh edit operations!'
            return lwsdk.AFUNC_OK

        try:
            # --------------------------------------------------------------
            # Parse file structure
            # --------------------------------------------------------------
            vertline  = []
            polyline  = []
            uvMaps    = []
            morphMaps = []
            weightMaps = []

            count = 0
            for line in lines:
                if line.startswith("VERTICES:"):
                    vertline.append([
                        int(line.strip().split(":")[1].strip()),
                        count
                    ])
                if line.startswith("POLYGONS:"):
                    polyline.append([
                        int(line.strip().split(":")[1].strip()),
                        count
                    ])
                if line.startswith("UV:"):
                    uvMaps.append([line.strip().split(":")[1:], count])
                if line.startswith("MORPH"):
                    morphMaps.append([line.split(":")[1].strip(), count])
                if line.startswith("WEIGHT"):
                    weightMaps.append([line.split(":")[1].strip(), count])
                count += 1

            # --------------------------------------------------------------
            # Create Points
            # --------------------------------------------------------------
            points = []
            for verts in vertline:
                for i in xrange(verts[1] + 1, verts[1] + verts[0] + 1):
                    x = map(float, lines[i].split())
                    points.append(
                        mesh_edit_op.addPoint(
                            mesh_edit_op.state,
                            [x[0], x[1], x[2] * -1]
                        )
                    )

            # --------------------------------------------------------------
            # Create Polygons
            # --------------------------------------------------------------
            polys = []
            for polygons in polyline:
                for i in xrange(polygons[1] + 1, polygons[1] + polygons[0] + 1):
                    pts   = []
                    split = lines[i].split(";;")
                    surf  = split[1].strip()
                    polytype = split[2].strip()

                    for x in split[0].split(","):
                        pts.insert(0, points[int(x.strip())])

                    ptype = lwsdk.LWPOLTYPE_FACE
                    if polytype == "CCSS":
                        ptype = lwsdk.LWPOLTYPE_SUBD
                    elif polytype == "SUBD":
                        ptype = lwsdk.LWPOLTYPE_PTCH

                    polys.append(
                        mesh_edit_op.addPoly(
                            mesh_edit_op.state,
                            ptype,
                            None,
                            surf,
                            pts
                        )
                    )

            # --------------------------------------------------------------
            # Setup Weightmaps
            # --------------------------------------------------------------
            for weightMap in weightMaps:
                mesh_edit_op.vMapSelect(
                    mesh_edit_op.state,
                    weightMap[0],
                    lwsdk.LWVMAP_WGHT,
                    1
                )
                count = 0
                for point in points:
                    line_val = lines[weightMap[1] + 1 + count].strip()
                    if line_val != "None":
                        mesh_edit_op.pntVMap(
                            mesh_edit_op.state,
                            point,
                            lwsdk.LWVMAP_WGHT,
                            weightMap[0],
                            [float(line_val)]
                        )
                    count += 1

            # --------------------------------------------------------------
            # Set Morph Map Values
            # --------------------------------------------------------------
            for morphMap in morphMaps:
                mesh_edit_op.vMapSelect(
                    mesh_edit_op.state,
                    morphMap[0],
                    lwsdk.LWVMAP_MORF,
                    3
                )
                count = 0
                for point in points:
                    line_val = lines[morphMap[1] + 1 + count].strip()
                    if line_val != "None":
                        parts = line_val.split(" ")
                        mesh_edit_op.pntVMap(
                            mesh_edit_op.state,
                            point,
                            lwsdk.LWVMAP_MORF,
                            morphMap[0],
                            [
                                float(parts[0]),
                                float(parts[1]),
                                float(parts[2]) * -1
                            ]
                        )
                    count += 1

            # --------------------------------------------------------------
            # Set UV Map Values
            # --------------------------------------------------------------
            for uvMap in uvMaps:
                name = uvMap[0][0]
                total = int(uvMap[0][1])
                mesh_edit_op.vMapSelect(
                    mesh_edit_op.state,
                    name,
                    lwsdk.LWVMAP_TXUV,
                    2
                )
                count = 0
                for i in range(total):
                    split = lines[uvMap[1] + 1 + count].split(":")
                    # discontinuous: pos:PLY:poly:PNT:pnt
                    if len(split) > 3:
                        pos  = split[0].split(" ")
                        poly = polys[int(split[2])]
                        pnt  = points[int(split[4])]
                        mesh_edit_op.pntVPMap(
                            mesh_edit_op.state,
                            pnt,
                            poly,
                            lwsdk.LWVMAP_TXUV,
                            name,
                            [float(pos[0]), float(pos[1])]
                        )
                    else:
                        # continuous: pos:PNT:pnt
                        pos  = split[0].split(" ")
                        pnt  = points[int(split[2])]
                        mesh_edit_op.pntVMap(
                            mesh_edit_op.state,
                            pnt,
                            lwsdk.LWVMAP_TXUV,
                            name,
                            [float(pos[0]), float(pos[1])]
                        )
                    count += 1

        except:
            edit_op_result = lwsdk.EDERR_USERABORT
            raise
        finally:
            mesh_edit_op.done(mesh_edit_op.state, edit_op_result, 0)

        return lwsdk.AFUNC_OK


################################################################
#  Layout helper: paste into a new null in Layout              #
################################################################

class OD_LayoutPasteFromExternal(lwsdk.IGeneric):
    def __init__(self, context):
        super(OD_LayoutPasteFromExternal, self).__init__()

    def process(self, ga):
        lwsdk.command('AddNull ODCopy')
        lwsdk.command('ModCommand_OD_LWPasteFromExternal Layout')
        return lwsdk.AFUNC_OK


# --------------------------------------------------------------------------
# Server records
# --------------------------------------------------------------------------

ServerTagInfo_OD_LWCopyToExternal = [
    ("OD_LWCopyToExternal", lwsdk.SRVTAG_USERNAME | lwsdk.LANGID_USENGLISH),
    ("OD_LWCopyToExternal", lwsdk.SRVTAG_BUTTONNAME | lwsdk.LANGID_USENGLISH)
]

ServerTagInfo_OD_LWPasteFromExternal = [
    ("OD_LWPasteFromExternal", lwsdk.SRVTAG_USERNAME | lwsdk.LANGID_USENGLISH),
    ("OD_LWPasteFromExternal", lwsdk.SRVTAG_BUTTONNAME | lwsdk.LANGID_USENGLISH)
]

ServerTagInfo_OD_LayoutPasteFromExternal = [
    ("OD_LayoutPasteFromExternal", lwsdk.SRVTAG_USERNAME | lwsdk.LANGID_USENGLISH),
    ("OD_LayoutPasteFromExternal", lwsdk.SRVTAG_BUTTONNAME | lwsdk.LANGID_USENGLISH)
]

ServerRecord = {
    lwsdk.CommandSequenceFactory("OD_LWPasteFromExternal", OD_LWPasteFromExternal):
        ServerTagInfo_OD_LWPasteFromExternal,
    lwsdk.CommandSequenceFactory("OD_LWCopyToExternal", OD_LWCopyToExternal):
        ServerTagInfo_OD_LWCopyToExternal,
    lwsdk.GenericFactory("OD_LayoutPasteFromExternal", OD_LayoutPasteFromExternal):
        ServerTagInfo_OD_LayoutPasteFromExternal
}
