# Script version 1.0 

# TODO: Create script to automate cutting the "L" shape notches in the back of the sheathing.
# TODO: Need to understand when bumps are framed at Melvin vs after. Then change bumpEast() and bumpWest() cut the new copy of sheathing.
# TODO: Add logic for identifying bumps and catch all. See 8&L Panel 10014.
# TODO: Get new geometry selecting functions to work with more/all panels. See Panel 11010. Panel 11027 and 09035 works 100%!
# TODO: Brick Feature EM needs logic: If edge = min and max x or z, it's not need.
# TODO: Perimeter Above Sheathing is not needed for every panel.
# TODO: If there is a return, perimeter selection needs to change to include .5in brick feature and come from the bottom face of the panel.
# TODO: 'Perimeter Above Sheathing' needs work to include returns.
# TODO: Window Bevel needs to work with returns/edge of panels.
# TODO: 'Return EM' has not geometry selection function.


import adsk.core, adsk.fusion, math, re, subprocess, os, sys, time, webbrowser, traceback

def run(context):
    # Set global variables
    global app, ui, product, design, rootComp

    app = adsk.core.Application.get()
    ui = app.userInterface
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)
    rootComp = design.rootComponent

    # Array for collecting messages to display at the end of the script.
    global script_summary  
    script_summary = []

    # list of functions
    rotateBodiesToFront()       # Rotates the bodies around the Z axis so the front of the panel is the front view.
    moveBodiesToOrigin()        # Moves all bodies to the origin.
    openPDF()                   # Opens the PDF drawing for the panel.
    stockBody()                 # Create a stock body for Charles setup.
    changeUnits()               # Change units to inches.
    idStuds()                   # Identify and rename stud bodies.
    idTrack()                   # Identify and rename track bodies.
    idCRC()                     # Identify and rename CRC bodies.
    idFoam()                    # Identify and rename foam bodies.
    idSheathing()               # Identify and rename sheathing bodies.
    mergeSheathing()            # Merge all sheathing panels into one.
    modSheathing()              # Draws and extrudes "L" shape cutouts for the Melvin toolpath.
    idRotatedBump()             # Identify and rename bump bodies that are on an angle.
    idBump()                    # Identify and rename bump bodies.
    idWindows()                 # Identifies windows and determines if the window bevel toolpath is needed.
    idOrigin()                  # Identifies WCS X and Y axis from the stud and track bodies.
    idReturn()                  # Is there a return on the right side of the panel that would interfere with WCS?
    melvinOrigin()              # Creates a sketch and construction point for the Melvin WCS.
    charlesOrigin()             # Creates a sketch and construction point for the Charles WCS.
    camWorkspace()              # Create the cam workspace.
    melvinSetup()               # Create the Melvin setup.
    charlesSetup()              # Create the Charles setup.
    idThinFoam()                # Checks for thin foam and adjusts the adjust toolpath as necessary.
    showBodies()                # Makes bodies visible and ready for geometry selection.
    melvinGeometry()            # Selects the edges for the Melvin Perimeter and generates the toolpath.
    facingheadGeometry()        # Selects the top face(s) for the facing operation and generates the toolpath.
    perimeterGeometry()         # Selects the geometry for the  'Perimeter Ball nose EM' operation and generates it's toolpath.
    aboveSheathingGeometry()    # Selects the geometry for the  'Perimeter Above Sheathing' operation and generates it's toolpath.
    windowBevelGeometry()       # Selects the window bevel edge(s) and generates the toolpath.
    brickFeatureEMGeometry()
    brickFeatureFMGeometry()
    bumpCleanUpGeometry()       # Selects edges for the east bump clean up operation and generates its toolpaths.
    eastReturnEMGeometry()       # Selects faces for the east return End mill operation and generates its toolpath.
    eastReturnGeometry()        # Selects faces for the east return face operation and generates its toolpath.
    eastReturnBrickGeometry()
    westReturnEMGeometry()
    westReturnGeometry()
    westReturnBrickGeometry()
    errorDetection()            # Compares 'Frame' and 'Sheathing' and 'Foam' and 'Sheathing' to detect errors from Revit export.
    scriptSummary()             # Displays a summary at the end of the script.

def addMessage(msg):            # This function adds messages throughout the script to give a summary at the end.
    try:
        script_summary.append(f"\u2022 {msg}\n")
    except:
        ui.messageBox(f"addMessage(): failed:\n{traceback.format_exc()}")

def in_cm(x):                   # This function converts inches to cm.
    return x * 2.54

def in_mm(x):                   # This function converts inches to mm.
    return x * 25.4

def rotateBodiesToFront():
    try:
        # Ask the user to select a face
        selection = ui.selectEntity('Select a face to be the new front view.', 'Faces')
        if not selection:
            ui.messageBox("No face selected. Script stopped.")
            return

        face = selection.entity
        if not isinstance(face, adsk.fusion.BRepFace):
            ui.messageBox("Selected entity is not a valid BRepFace. Script stopped.")
            return

        # Get the normal vector of the face at its centroid
        success, current_normal = face.evaluator.getNormalAtPoint(face.centroid)
        if not success:
            ui.messageBox("Failed to get face normal. Script stopped.")
            return
        current_normal.normalize()

        # Define target normal (global negative Y)
        target_normal = adsk.core.Vector3D.create(0, -1, 0)

        # Compute rotation axis (perpendicular to both)
        rotation_axis = current_normal.crossProduct(target_normal)

        # Check for edge cases: already aligned or exactly opposite
        if rotation_axis.length < .002:
            dot = current_normal.dotProduct(target_normal)
            if dot > 0:
                #ui.messageBox("Face is already aligned with target. No rotation needed.")
                return
            else:
                # Opposite direction: rotate 180 degrees around global Z
                rotation_axis = adsk.core.Vector3D.create(0, 0, 1)
                angle = math.pi
        else:
            rotation_axis.normalize()
            # Rotation angle
            dot = max(-1.0, min(1.0, current_normal.dotProduct(target_normal)))
            angle = math.acos(dot)

        # Compute rotation origin (center of the root component)
        bbox = rootComp.boundingBox
        rotation_origin = adsk.core.Point3D.create(
            (bbox.minPoint.x + bbox.maxPoint.x) / 2.0,
            (bbox.minPoint.y + bbox.maxPoint.y) / 2.0,
            (bbox.minPoint.z + bbox.maxPoint.z) / 2.0
        )

        # Create transformation matrix
        transform = adsk.core.Matrix3D.create()
        transform.setToRotation(angle, rotation_axis, rotation_origin)

        # Collect all bodies to move
        bodies = adsk.core.ObjectCollection.create()
        for body in rootComp.bRepBodies:
            bodies.add(body)
        if bodies.count == 0:
            ui.messageBox("No bodies found in root component. Script stopped.")
            return

        # Apply the move
        move_input = rootComp.features.moveFeatures.createInput(bodies, transform)
        rootComp.features.moveFeatures.add(move_input)

        # Optional: reset the camera to front view
        camera = app.activeViewport.camera
        camera.upVector = adsk.core.Vector3D.create(0, 0, 1)   # Z is up
        camera.viewOrientation = adsk.core.ViewOrientations.FrontViewOrientation
        app.activeViewport.camera = camera
        app.activeViewport.fit()

    except:
        ui.messageBox(f"rotateBodiesToFront() failed:\n{traceback.format_exc()}")

def moveBodiesToOrigin():
    try:
        # Find 'Body1' and rename as "Exterior"
        body1 = None
        for body in rootComp.bRepBodies:
            if body.name == 'Body1':
                body1 = body
                break       
            
        # Rename the selected body
        body1.name = "Exterior"

        # Create a vector representing panel coordinate system
        panelCoord = adsk.core.Vector3D.create(body1.boundingBox.maxPoint.x, body1.boundingBox.maxPoint.y, body1.boundingBox.maxPoint.z)

        # Create a vector representing the translation needed to move to the origin (0,0,0)
        translationVector = adsk.core.Vector3D.create(-panelCoord.x, -panelCoord.y, -panelCoord.z)

        # Create a transformation matrix from the translation vector
        transformMatrix = adsk.core.Matrix3D.create()
        transformMatrix.translation = translationVector

        # Get the move features collection
        moveFeatures = rootComp.features.moveFeatures

        # Create an ObjectCollection to hold ALL bodies to be moved
        bodiesToMove = adsk.core.ObjectCollection.create()
        for body in rootComp.bRepBodies:
            bodiesToMove.add(body)

        # Create a move input with all bodies and the transformation matrix
        moveInput = moveFeatures.createInput(bodiesToMove, transformMatrix)

        # Add the move feature to the design
        moveFeatures.add(moveInput)

        # Fit the view to the new position of the assembly
        app.activeViewport.fit()
    except:
        ui.messageBox(f"moveBodiesToOrigin(): failed:\n{traceback.format_exc()}")
    
def stockBody():
    try: 
        # Find Exterior Body 
        target_body_name = "Exterior"
        body_to_copy = None
        for body in rootComp.bRepBodies:
            if body.name == target_body_name:
                body_to_copy = body
                break

        # Copy the body
        copy_paste_features = rootComp.features.copyPasteBodies
        bodies_to_copy_collection = adsk.core.ObjectCollection.create()
        bodies_to_copy_collection.add(body_to_copy)
        new_bodies_collection = copy_paste_features.add(bodies_to_copy_collection).bodies # Get the 'bodies' collection directly

        new_body = new_bodies_collection.item(0) # Get the first (and likely only) copied body

        # Move the new copy -.4 in the Y-axis (will be machine +Z axis)
        transform = adsk.core.Matrix3D.create()
        transform.translation = adsk.core.Vector3D.create(0, -0.4, 0)

        move_features = rootComp.features.moveFeatures
        bodies_to_move = adsk.core.ObjectCollection.create()
        bodies_to_move.add(new_body)

        move_feature_input = move_features.createInput(bodies_to_move, transform)
        move_features.add(move_feature_input)

        # Rename the new copy to "Stock"
        new_body.name = "Stock"
        #new_body.appearance = design.appearances.itemByName('Paint - Enamel Glossy (Green)')

        # Hide the stock body
        adsk.fusion.Design.cast(app.activeProduct).rootComponent.bRepBodies.itemByName('Stock').isVisible = False
    except:
        ui.messageBox(f"stockBody(): failed:\n{traceback.format_exc()}")

def changeUnits():
    try:
        # Get the UnitManager from the active design
        unitMgr = design.unitsManager

        # Get the current default length units as a string
        current_units_str = unitMgr.defaultLengthUnits.lower()

        # Define the commands to change units to inches
        txtCmds = [
            u"NaFusionUI.ChangeActiveUnitsCmd ",
            u"Commands.SetString infoUnitsType InchImperial",
            u"NuCommands.CommitCmd",
        ]
        if current_units_str != 'in':
            [app.executeTextCommand(cmd) for cmd in txtCmds]
    except:
        ui.messageBox(f"changeUnits(): failed:\n{traceback.format_exc()}")

def idFoam():
    try:
        foam_dim = in_cm(3.0) 
        tolerance = in_cm(.001)    
        foam_bodies = []

        # Look for body that is 3" thick 
        global foamResult
        foamResult = False
        
        # Iterate through all bodies in the root component.
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            
            if (abs(width - foam_dim) < tolerance):
                # Add the body to the list and rename it
                foam_bodies.append(body)
                body.name = "Foam"

        # Checks for body named "Foam"        
        if any("Foam" in body.name for body in rootComp.bRepBodies):
            foamResult = True
    except:
        ui.messageBox(f"identifyFoam(): failed:\n{traceback.format_exc()}")

def idCRC():
    tolerance = in_cm(.001)   
    try:
        # Bump stud 6" X 2.5"
        frame_stud_w_dim = in_cm(1.5)
        frame_stud_h_dim = in_cm(.5)

        frame_stud_bodies = [] 

        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox

            length = boundingBox.maxPoint.z - boundingBox.minPoint.z
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            
            if (abs(length - frame_stud_h_dim) < tolerance and
                abs(width - frame_stud_w_dim) < tolerance):
                # Add the body to the list and rename it
                frame_stud_bodies.append(body)
                body.name = "CRC"
    except:
        ui.messageBox(f"idCRC(): failed:\n{traceback.format_exc()}")

def idStuds():
    try:
        stud_dim = in_cm(6.0)
        tolerance = in_cm(.001)
        stud_bodies = [] 
        
        # Iterate through all bodies in the root component.
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            
            if (abs(width - stud_dim) < tolerance):
                # Add the body to the list and rename it
                stud_bodies.append(body)
                body.name = "Stud"
    except:
        ui.messageBox(f"identifyStuds(): failed:\n{traceback.format_exc()}")
            
def idTrack():
    try:
        track_dim = in_cm(6.143)
        tolerance = in_cm(.001)
        track_bodies = [] 
        
        # Iterate through all bodies in the root component.
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            
            if (abs(width - track_dim) < tolerance) :
                # Add the body to the list and rename it
                track_bodies.append(body)
                body.name = "Track"
    except:
        ui.messageBox(f"identifyTrack(): failed:\n{traceback.format_exc()}")

def idSheathing():
    try:
        sheathing_thickness = in_cm(0.625)
        tolerance = in_cm(0.001)

        # --- Step 1: Rename all bodies as "Sheathing" ---
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            if abs(width - sheathing_thickness) < tolerance:
                body.name = "SheathingPanel"
                body.isVisible = False
    except:
        ui.messageBox(f"idSheathing(): failed:\n{traceback.format_exc()}")

def mergeSheathing():
    try:
        # Make copies and rename them "SheathingCopy"
        copy_bodies = []
        for body in rootComp.bRepBodies:
            if body.name.startswith("SheathingPanel"):
                body_copy = body.copyToComponent(rootComp)
                body_copy.name = "SheathingCopy"
                copy_bodies.append(body_copy)

        # Keep merging until only one "Sheathing" body remains
        for _ in range(2):  # Run up to twice
            bodies_to_merge = []

            for body in rootComp.bRepBodies:
                if body.name.startswith("SheathingCopy"):
                    bodies_to_merge.append(body)

            if len(bodies_to_merge) <= 1:
                break  # Done merging

            combineFeatures = rootComp.features.combineFeatures
            targetBody = bodies_to_merge[0]
            for toolBody in bodies_to_merge[1:]:
                toolBodies = adsk.core.ObjectCollection.create()
                toolBodies.add(toolBody)
                combineInput = combineFeatures.createInput(targetBody, toolBodies)
                combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
                combineFeature = combineFeatures.add(combineInput)
                targetBody = combineFeature.bodies.item(0)
                targetBody.name = "Sheathing"

    except:
        ui.messageBox(f"mergeSheathing(): failed:\n{traceback.format_exc()}")

def modSheathing():
    try:
        # ----- PARAMETERS -----
        BODY_NAME = "Sheathing"
        TARGET_Y_IN = -6.0
        TOL_IN = 0.001
        LONG_LEG_IN = 3
        THICKNESS_IN = 1
        EXTRUDE_DISTANCE = '-.2 in'

        # Convert to cm
        target_y_cm = TARGET_Y_IN * 2.54
        tol_cm = TOL_IN * 2.54
        long_leg_cm = LONG_LEG_IN * 2.54
        thickness_cm = THICKNESS_IN * 2.54
        target_body = None

        # ----- FIND SHEATHING BODY -----
        for body in rootComp.bRepBodies:
            if body.name == BODY_NAME:
                target_body = body
                break
        if not target_body:
            addMessage(f'Body "{BODY_NAME}" not found.')
            return

        # ----- FIND XZ-PARALLEL FACE AT Y=-6in -----
        selected_face = None
        for face in target_body.faces:
            normal = face.geometry.normal
            if abs(abs(normal.y) - 1.0) < 0.001:  # XZ-parallel
                bb = face.boundingBox
                y_center = (bb.minPoint.y + bb.maxPoint.y) / 2.0
                if abs(y_center - target_y_cm) < tol_cm:
                    selected_face = face
                    break
        if not selected_face:
            ui.messageBox(f'No XZ-parallel face found at Y = {TARGET_Y_IN}"', 'Error')
            return

        parent_comp = selected_face.body.parentComponent

        # ----- CREATE SKETCH ON SELECTED FACE -----
        sketches = parent_comp.sketches
        sketch = sketches.add(selected_face)
        sketch.name = "Corner L-Features"

        # ----- COLLECT VERTICES & BOUNDING BOX -----
        unique_vertices = {}
        min_sketch_x = float('inf')
        max_sketch_x = float('-inf')
        min_sketch_y = float('inf')
        max_sketch_y = float('-inf')

        for loop in selected_face.loops:
            for edge in loop.edges:
                for vertex in [edge.startVertex, edge.endVertex]:
                    if vertex:
                        key = str(vertex.geometry.asArray())
                        if key not in unique_vertices:
                            unique_vertices[key] = vertex
                        sp = sketch.modelToSketchSpace(vertex.geometry)
                        min_sketch_x = min(min_sketch_x, sp.x)
                        max_sketch_x = max(max_sketch_x, sp.x)
                        min_sketch_y = min(min_sketch_y, sp.y)
                        max_sketch_y = max(max_sketch_y, sp.y)

        if not unique_vertices:
            ui.messageBox('No corners (vertices) found on the selected face.', 'No Corners')
            return

        # ----- DRAW L-SHAPES -----
        tolerance = 0.0001
        features_drawn_count = 0

        for vertex_key, vertex in unique_vertices.items():
            sp = sketch.modelToSketchSpace(vertex.geometry)
            l_points = []

            is_min_x = abs(sp.x - min_sketch_x) < tolerance
            is_max_x = abs(sp.x - max_sketch_x) < tolerance
            is_min_y = abs(sp.y - min_sketch_y) < tolerance
            is_max_y = abs(sp.y - max_sketch_y) < tolerance

            # Define L-shape points
            if is_min_x and is_min_y:  # Bottom-left
                l_points = [
                    adsk.core.Point3D.create(sp.x, sp.y, 0),
                    adsk.core.Point3D.create(sp.x + long_leg_cm, sp.y, 0),
                    adsk.core.Point3D.create(sp.x + long_leg_cm, sp.y + thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x + thickness_cm, sp.y + thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x + thickness_cm, sp.y + long_leg_cm, 0),
                    adsk.core.Point3D.create(sp.x, sp.y + long_leg_cm, 0)
                ]
            elif is_max_x and is_min_y:  # Bottom-right
                l_points = [
                    adsk.core.Point3D.create(sp.x, sp.y, 0),
                    adsk.core.Point3D.create(sp.x - long_leg_cm, sp.y, 0),
                    adsk.core.Point3D.create(sp.x - long_leg_cm, sp.y + thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x - thickness_cm, sp.y + thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x - thickness_cm, sp.y + long_leg_cm, 0),
                    adsk.core.Point3D.create(sp.x, sp.y + long_leg_cm, 0)
                ]
            elif is_min_x and is_max_y:  # Top-left
                l_points = [
                    adsk.core.Point3D.create(sp.x, sp.y, 0),
                    adsk.core.Point3D.create(sp.x + long_leg_cm, sp.y, 0),
                    adsk.core.Point3D.create(sp.x + long_leg_cm, sp.y - thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x + thickness_cm, sp.y - thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x + thickness_cm, sp.y - long_leg_cm, 0),
                    adsk.core.Point3D.create(sp.x, sp.y - long_leg_cm, 0)
                ]
            elif is_max_x and is_max_y:  # Top-right
                l_points = [
                    adsk.core.Point3D.create(sp.x, sp.y, 0),
                    adsk.core.Point3D.create(sp.x - long_leg_cm, sp.y, 0),
                    adsk.core.Point3D.create(sp.x - long_leg_cm, sp.y - thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x - thickness_cm, sp.y - thickness_cm, 0),
                    adsk.core.Point3D.create(sp.x - thickness_cm, sp.y - long_leg_cm, 0),
                    adsk.core.Point3D.create(sp.x, sp.y - long_leg_cm, 0)
                ]
            else:
                continue  # skip non-corner vertices

            if l_points:
                # Draw L polygon and close it
                lines = sketch.sketchCurves.sketchLines
                for i in range(len(l_points)):
                    lines.addByTwoPoints(l_points[i], l_points[(i + 1) % len(l_points)])

                features_drawn_count += 1

        # ----- COLLECT VALID L-SHAPE PROFILES -----
        l_profiles = []
        for prof in sketch.profiles:
            bb = prof.boundingBox
            width = bb.maxPoint.x - bb.minPoint.x
            height = bb.maxPoint.y - bb.minPoint.y
            if width <= long_leg_cm + 0.01 and height <= long_leg_cm + 0.01:
                l_profiles.append(prof)

        if not l_profiles:
            addMessage('No valid L-shape profiles found for extrude cut.')
            return

        # ----- EXTRUDE CUT EACH L-SHAPE -----
        extrudes = parent_comp.features.extrudeFeatures
        for prof in l_profiles:
            distance_input = adsk.core.ValueInput.createByString(EXTRUDE_DISTANCE)
            extrude_input = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
            extrude_input.setDistanceExtent(False, distance_input)
            extrudes.add(extrude_input)

    except:
        ui.messageBox(f"modSheathing(): failed:\n{traceback.format_exc()}")

def idRotatedBump():            # This function works but is sloppy and needs to be cleaned up.
    global eastBump, westBump
    eastBump = False
    westBump = False  
    try:
        if not design: return

        rootComp = design.rootComponent
        xzPlaneNormal = adsk.core.Vector3D.create(0, 1, 0)
        
        minAngleRad = math.radians(2.0)
        maxAngleRad = math.radians(82.0)

        rotated_bodies = []

        for body in rootComp.bRepBodies:
            if body.name.startswith("Body"):
                
                hasRotatedFace = False
                
                for face in body.faces:
                    normal = None
                    surface = face.geometry

                    if surface.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType:
                        normal = adsk.core.Plane.cast(surface).normal
                    else:
                        try:
                            bBox = face.boundingBox
                            center = adsk.core.Point3D.create(
                                (bBox.minPoint.x + bBox.maxPoint.x) / 2,
                                (bBox.minPoint.y + bBox.maxPoint.y) / 2,
                                (bBox.minPoint.z + bBox.maxPoint.z) / 2
                            )
                            success, _, paramU, paramV = face.getClosestPoint(center)
                            if success:
                                normal = face.getNormalAtParameter(paramU, paramV)
                        except:
                            pass # Ignore faces where normal calculation fails

                    if normal:
                        dotProduct = normal.dotProduct(xzPlaneNormal)
                        dotProduct = max(-1.0, min(1.0, dotProduct)) # Clamp for precision

                        angleRad = math.acos(abs(dotProduct))
                        
                        if angleRad >= minAngleRad and angleRad <= maxAngleRad:
                            hasRotatedFace = True
                            break 

                if hasRotatedFace:
                    body.name = "Unknown"
                    rotated_bodies.append(body.name)

        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        unknown = [body for body in rootComp.bRepBodies if body.name.startswith("Unknown")]
        # Loop through all "Unknown" bodies and expand bounds
        for body in unknown:
            box = body.boundingBox
            min_x = min(min_x, box.minPoint.x)
            min_y = min(min_y, box.minPoint.y)
            min_z = min(min_z, box.minPoint.z)
            max_x = max(max_x, box.maxPoint.x)
            max_y = max(max_y, box.maxPoint.y)
            max_z = max(max_z, box.maxPoint.z)

            unknown_max_point = adsk.core.Point3D.create(max_x, max_y, max_z) # This is used to identify a west return. No affect to the origin.

            if abs(unknown_max_point.y) > in_cm(6):
                body.name = "RotatedBump"

        if any("RotatedBump" in body.name for body in rootComp.bRepBodies):
            # Measure distance from origin to the Bump body
            bump = next((b for b in rootComp.bRepBodies if b.name == "RotatedBump"), None)
            if bump:
                bbox = bump.boundingBox
                # Measure from the origin (Y=0) to the closest Y point on the body
                
                if abs(bbox.minPoint.x) < in_cm(20):
                    eastBump = True
                    bumpEast()
                else:
                    westBump = True
                    bumpWest()
            addMessage("A Rotated bump has been detected. Adjust toolpaths accordingly.")
    except:
        ui.messageBox('identifyRotatedBump Failed:\n{}'.format(traceback.format_exc()))

def idBump():
    try:
        global eastBump, westBump
        eastBump = False
        westBump = False

        bump_track_dim = in_cm(6.086)
        tolerance = in_cm(.001)  
        bump_track_bodies = [] 
        
        # Iterate through all bodies in the root component.
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox

            length = boundingBox.maxPoint.x - boundingBox.minPoint.x
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            height = boundingBox.maxPoint.z - boundingBox.minPoint.z
            
            if (abs(length - bump_track_dim) < tolerance) :
                # Add the body to the list and rename it
                bump_track_bodies.append(body)
                body.name = "Bump"

        # Bump stud 6" X 2.5"
        bump_stud_w_dim = in_cm(6.0)
        bump_stud_h_dim = in_cm(2.5)

        bump_stud_bodies = [] 

        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox

            length = boundingBox.maxPoint.x - boundingBox.minPoint.x
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            
            if (abs(length - bump_stud_w_dim) < tolerance and
                abs(width - bump_stud_h_dim) < tolerance):
                # Add the body to the list and rename it
                bump_stud_bodies.append(body)
                body.name = "Bump"

        # Measure distance from origin to the Bump body
        bump = next((b for b in rootComp.bRepBodies if b.name == "Bump"), None)
        if bump:
            bbox = bump.boundingBox
            # Measure from the origin (Y=0) to the closest Y point on the body
            if abs(bbox.minPoint.x) < in_cm(20):
                eastBump = True
                bumpEast()
            else:
                westBump = True
                bumpWest()
            addMessage("A bump has been detected. Adjust toolpaths accordingly.")

        if not any("Stud" in body.name for body in rootComp.bRepBodies):
             # Ask User if the panel has a return
            question_text = """Frame components could not be detected. \nIs there a East bump? (right side)""" 
            button_type = adsk.core.MessageBoxButtonTypes.YesNoButtonType
            warning_icon = adsk.core.MessageBoxIconTypes.WarningIconType
            eastBumpQ = ui.messageBox(question_text, "Warning", button_type, warning_icon)
            if eastBumpQ == adsk.core.DialogResults.DialogYes:
                eastBump = True
                bumpEast()
            else:
                question_text = """Frame components could not be detected. \nIs there a West bump? (left side)""" 
                button_type = adsk.core.MessageBoxButtonTypes.YesNoButtonType
                warning_icon = adsk.core.MessageBoxIconTypes.WarningIconType
                westBumpQ = ui.messageBox(question_text, "Warning", button_type, warning_icon)
                if westBumpQ == adsk.core.DialogResults.DialogYes:
                    westBump = True
                    bumpWest()

    except:
        ui.messageBox(f"idBump(): failed:\n{traceback.format_exc()}")

def idWindows():
    global windows
    windows = False

    try:
        target_y_cm = in_cm(-6.625)
        tol_y = in_cm(0.001)  # small tolerance for Y comparison

        ext_body = rootComp.bRepBodies.itemByName("Exterior")
        if not ext_body:
            return

        window_start_cm = None
        window_end_cm = None
        window_z_top_cm = None

        # --- STEP 1: Find window face and X/Z extents ---
        for face in ext_body.faces:
            geom = face.geometry
            if not isinstance(geom, adsk.core.Plane):
                continue

            normal = geom.normal
            origin = geom.origin

            # ZX-plane (normal along Y)
            if abs(abs(normal.y) - 1.0) > 1e-3:
                continue

            if abs(origin.y - target_y_cm) > in_cm(0.01):
                continue

            for loop in face.loops:
                if loop.isOuter:
                    continue

                min_x = float('inf')
                max_x = float('-inf')
                min_z = float('-inf')

                for edge in loop.edges:
                    bbox = edge.boundingBox
                    min_x = min(min_x, bbox.minPoint.x)
                    max_x = max(max_x, bbox.maxPoint.x)
                    min_z = max(min_z, bbox.minPoint.z)

                window_start_cm = min_x
                window_end_cm = max_x
                window_z_bottom_cm = min_z - 2.54

        if window_start_cm is None:
            return  # no window found

        min_z = ext_body.boundingBox.minPoint.z + 2.54

        # --- STEP 2: Check for edges under window ---
        for edge in ext_body.edges:
            v1 = edge.startVertex.geometry
            v2 = edge.endVertex.geometry

            # Both vertices at Y = target_y_cm
            if v1.y == target_y_cm and v2.y == target_y_cm:
                continue

            # Edge within X range of window
            # edge_min_x = min(v1.x, v2.x)
            # edge_max_x = max(v1.x, v2.x)
            # if edge_max_x < window_start_cm or edge_min_x > window_end_cm:
            #     continue

            if v1.x == v2.x:
                continue

            # Edge within Z range under window
            # edge_min_z = min(v1.z, v2.z)
            # edge_max_z = max(v1.z, v2.z)
            # if edge_max_z < min_z or edge_min_z > window_z_bottom_cm:
            #     windows = True
            
            if v1.z > min_z and v1.z < window_z_bottom_cm:
                #ui.messageBox(f"Edge: v1.z = {(v1.z / 2.54)}, v2.z = {(v2.z / 2.54)}")
                windows = True

    except:
        ui.messageBox('idWindows() Failed:\n{}'.format(traceback.format_exc()))

def bumpEast():                 # This function cuts the "Stock" and "Foam" bodies to prevent the facinghead from interfering with the bump.
    try:
        # Hide all bodies in the root component
        for body in rootComp.bRepBodies:
            body.isVisible = False

        for body in rootComp.bRepBodies:
            if body.name == "Sheathing":
                body.isVisible = True
                stock_body = body

        # Show Stock body
        for body in rootComp.bRepBodies:
            if body.name == "Stock":
                body.isVisible = True
                stock_body = body

        # Sketch on XZ plane (front view)
        sketches = rootComp.sketches
        xzPlane = rootComp.xZConstructionPlane
        sketch = sketches.add(xzPlane)

        # Offset from bump
        x_offset = 32 #inches
        x_offset = (x_offset * 2.54 * -1) # convert to CM and change to be negative

        # Draw rectangle in sketch plane coordinates (X = horizontal, Y = vertical)
        lines = sketch.sketchCurves.sketchLines
        lines.addTwoPointRectangle(adsk.core.Point3D.create(x_offset, 0, 0),
                                   adsk.core.Point3D.create(0, 550, 0))

        extrudes = rootComp.features.extrudeFeatures
        extrudeInput = extrudes.createInput(sketch.profiles[0], adsk.fusion.FeatureOperations.CutFeatureOperation)
        distance = adsk.core.ValueInput.createByReal(-15 * 2.54)
        extrudeInput.setDistanceExtent(False, distance)
        extrudes.add(extrudeInput)

        # Hide Stock body
        stock_body.isVisible = False

        # Cut foam body
        if foamResult == True:
            # Show Stock body
            for body in rootComp.bRepBodies:
                if body.name == "Foam":
                    body.isVisible = True
                    foam_body = body

            # Sketch on XZ plane (front view)
            sketches = rootComp.sketches
            xzPlane = rootComp.xZConstructionPlane
            sketch = sketches.add(xzPlane)

            # Offset from bump
            x_offset = 8 #inches
            x_offset = (x_offset * 2.54 * -1) # convert to CM and change to be negative

            # Draw rectangle in sketch plane coordinates (X = horizontal, Y = vertical)
            lines = sketch.sketchCurves.sketchLines
            lines.addTwoPointRectangle(adsk.core.Point3D.create(x_offset, 0, 0),
                                    adsk.core.Point3D.create(0, 550, 0))

            extrudes = rootComp.features.extrudeFeatures
            extrudeInput = extrudes.createInput(sketch.profiles[0], adsk.fusion.FeatureOperations.CutFeatureOperation)
            distance = adsk.core.ValueInput.createByReal(-15 * 2.54)
            extrudeInput.setDistanceExtent(False, distance)
            extrudes.add(extrudeInput)

            # Hide Foam body
            foam_body.isVisible = False
        
    except:
        ui.messageBox(f"bumpEast(): failed:\n{traceback.format_exc()}")

def bumpWest():                 # This function cuts the "Stock" and "Foam" bodies to prevent the facinghead from interfering with the bump.
    try:
        # Hide all bodies in the root component
        for body in rootComp.bRepBodies:
            body.isVisible = False

        for body in rootComp.bRepBodies:
            if body.name == "Sheathing":
                body.isVisible = True
                stock_body = body

        # Hide all bodies in the root component
        for body in rootComp.bRepBodies:
            if body.name == "Stock":
                body.isVisible = True
                stock_body = body

        # Sketch on XZ plane (front view)
        sketches = rootComp.sketches
        xzPlane = rootComp.xZConstructionPlane
        sketch = sketches.add(xzPlane)
    
        # Bounding box of the Stock body
        bb = stock_body.boundingBox
        min_x = bb.minPoint.x

        # Offset from bump
        x_offset = 32 #inches
        x_offset = (x_offset * 2.54 * -1) # convert to CM and change to be negative

        # Draw rectangle in sketch plane coordinates (X = horizontal, Y = vertical)
        lines = sketch.sketchCurves.sketchLines
        lines.addTwoPointRectangle(adsk.core.Point3D.create(min_x, 0, 0),
                                   adsk.core.Point3D.create((min_x - x_offset), 550, 0))

        extrudes = rootComp.features.extrudeFeatures
        extrudeInput = extrudes.createInput(sketch.profiles[0], adsk.fusion.FeatureOperations.CutFeatureOperation)
        distance = adsk.core.ValueInput.createByReal(-15 * 2.54)
        extrudeInput.setDistanceExtent(False, distance)
        extrudes.add(extrudeInput)

        stock_body.isVisible = False

        # Cut foam body
        if foamResult == True:
            # Show Stock body
            for body in rootComp.bRepBodies:
                if body.name == "Foam":
                    body.isVisible = True
                    foam_body = body

            # Sketch on XZ plane (front view)
            sketches = rootComp.sketches
            xzPlane = rootComp.xZConstructionPlane
            sketch = sketches.add(xzPlane)

            # Offset from bump
            x_offset = 8 #inches
            x_offset = (x_offset * 2.54 * -1) # convert to CM and change to be negative

            # Draw rectangle in sketch plane coordinates (X = horizontal, Y = vertical)
            lines = sketch.sketchCurves.sketchLines
            lines.addTwoPointRectangle(adsk.core.Point3D.create(min_x, 0, 0),
                                   adsk.core.Point3D.create((min_x - x_offset), 550, 0))

            extrudes = rootComp.features.extrudeFeatures
            extrudeInput = extrudes.createInput(sketch.profiles[0], adsk.fusion.FeatureOperations.CutFeatureOperation)
            distance = adsk.core.ValueInput.createByReal(-15 * 2.54)
            extrudeInput.setDistanceExtent(False, distance)
            extrudes.add(extrudeInput)

            # Hide Foam body
            foam_body.isVisible = False
    
    except:
        ui.messageBox(f"bumpWest(): failed:\n{traceback.format_exc()}")

def openPDF():
    try:
        # Normalize inputs for case-insensitive searching
        current_doc_name = app.activeDocument.name
        panel_number = re.sub(r'\s*[vV]\d+', '', current_doc_name) 
        
        directory_path = r"C:\Users\ahughes\Dropbox\UPLOADED TO BIM360"
        # os.walk is efficient for traversing directory trees
        for root, _, files in os.walk(directory_path):
            for filename in files:
                # Check if the file is a PDF and contains the part number
                if filename.lower().endswith('.pdf') and panel_number in filename.lower():
                    full_path = os.path.join(root, filename)
                    os.startfile(full_path)

        # Search PDF for important details.
        search_terms = [
                        "P-203",  # This is the 1 inch cut above the sheathing for corner brick.
                        # Add more here if needed
                        ]
        pdf_checker = r"C:\Users\ahughes\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts\PanelStartup\pdf_text_search.py"

        # # Build the command
        python_exe = r"C:\Users\ahughes\AppData\Local\Programs\Python\Python313\pythonw.exe"
        cmd = [python_exe, pdf_checker, directory_path, panel_number] + search_terms
        subprocess.run(cmd)

    except:
        ui.messageBox(f"openPDF(): failed:\n{traceback.format_exc()}")
        
def idOrigin():
    try:
        # Find the edge of the stud body for our WCS
        studs = [body for body in rootComp.bRepBodies if body.name.startswith("Stud")]

        # Initialize min and max values
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        # Loop through all "Stud" bodies and expand bounds
        for body in studs:
            box = body.boundingBox
            min_x = min(min_x, box.minPoint.x)
            min_y = min(min_y, box.minPoint.y)
            min_z = min(min_z, box.minPoint.z)
            max_x = max(max_x, box.maxPoint.x)
            max_y = max(max_y, box.maxPoint.y)
            max_z = max(max_z, box.maxPoint.z)

        # Create Point3D objects for easy use later
        global stud_max_point, stud_min_point
        stud_max_point = adsk.core.Point3D.create(max_x, max_y, max_z) # This is used to identify a east return, which affect the origin position.
        stud_min_point = adsk.core.Point3D.create(min_x, min_y, min_z) # This is used to identify a west return. No affect to the origin.
        
        # Find the edge of the stud body for our WCS
        track = [body for body in rootComp.bRepBodies if body.name.startswith("Track")]

        # Initialize min and max values
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
        max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

        # Loop through all "Stud" bodies and expand bounds
        for body in track:
            box = body.boundingBox
            min_x = min(min_x, box.minPoint.x)
            min_y = min(min_y, box.minPoint.y)
            min_z = min(min_z, box.minPoint.z)
            max_x = max(max_x, box.maxPoint.x)
            max_y = max(max_y, box.maxPoint.y)
            max_z = max(max_z, box.maxPoint.z)

        # Create Point3D objects for easy use later
        global track_max_point, track_min_point
        track_max_point = adsk.core.Point3D.create(max_x, max_y, max_z)
        track_min_point = adsk.core.Point3D.create(min_x, min_y, min_z)
        
    except:
        ui.messageBox(f"{traceback.format_exc()}", "identifyOrigin(): failed!")

def idReturn():
    global east_return_result, west_return_result
    east_return_result = adsk.core.DialogResults.DialogNo
    west_return_result = adsk.core.DialogResults.DialogNo
 
    # The script can only identify a return if there are "Stud" bodies in the model.
    # Here we confirm if there are "Stud" bodies. If a Stud is not found, the script ask the user to refer to the panel drawing and select Yes or No.
    try:
        studs = [body for body in rootComp.bRepBodies if body.name.startswith("Stud")]
        if not studs:
            # Ask User if the panel has a return
            question_text = """Frame bodies were not detected. Check the drawing for a return on the right hand side of the panel.\n 
        \u2022 'No' will place the coordinate system at 0.0625\" from the Exterior body.\n
        \u2022 'Yes' will place the coordinate system at 4.6875\" from the Exterior body.""" 
            button_type = adsk.core.MessageBoxButtonTypes.YesNoButtonType
            warning_icon = adsk.core.MessageBoxIconTypes.WarningIconType
            east_return_result = ui.messageBox(question_text, "Warning", button_type, warning_icon)
            west_return_result = ui.messageBox("Is there a west return? (Left side of the panel)", "Warning", button_type, warning_icon)
            addMessage("Frame bodies were not detected.")


            return None, None

        if abs(stud_max_point.x / 2.54) > 4:
            east_return_result = adsk.core.DialogResults.DialogYes
        
        # This section is for identifying a west return.
        min_x, min_y, min_z = float('inf'), float('inf'), float('inf')

        for body in rootComp.bRepBodies:
            if body.name == "Exterior":  
                box = body.boundingBox
                min_x = min(min_x, box.minPoint.x)
                min_y = min(min_y, box.minPoint.y)
                min_z = min(min_z, box.minPoint.z)
        
        exterior_min_point = adsk.core.Point3D.create(min_x, min_y, min_z)

        if (abs(exterior_min_point.x / 2.54) - abs(stud_min_point.x / 2.54)) > 4:
            west_return_result = adsk.core.DialogResults.DialogYes
    
    except:
        ui.messageBox(f"{traceback.format_exc()}", "isReturn(): failed!")

def melvinOrigin():
    try:
        body = rootComp.bRepBodies.item(0)
        
        # Compute overall bounding box extents
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        min_z, max_z = float('inf'), float('-inf')

        for face in body.faces:
            boundingBox = face.boundingBox
            min_x = min(min_x, boundingBox.minPoint.x)
            max_x = max(max_x, boundingBox.maxPoint.x)
            min_y = min(min_y, boundingBox.minPoint.y)
            max_y = max(max_y, boundingBox.maxPoint.y)
            min_z = min(min_z, boundingBox.minPoint.z)
            max_z = max(max_z, boundingBox.maxPoint.z)
        
        # Back-top-right corner
        corner_x, corner_y, corner_z = max_x, max_y, max_z

        # Offsets
        offset_x = corner_x - in_cm(0.0625)
        offset_y = corner_y - in_cm(6.0)
        offset_z = corner_z - in_cm(0.0625)

        if any("Stud" in body.name for body in rootComp.bRepBodies):
            offset_x = stud_max_point.x

        if any("Track" in body.name for body in rootComp.bRepBodies):
            offset_z = track_max_point.z

        # This tries to set the WCS X axis to location identified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if east_return_result == adsk.core.DialogResults.DialogYes:
            if any("Stud" in body.name for body in rootComp.bRepBodies):
                offset_x = stud_max_point.x
                difference = abs((abs(max_x)) - (abs(stud_max_point.x)) / 2.54)
                if not abs(difference - 0.063) >= .002 or abs(difference - 4.625 >= .002): #Probably need sheathing width as well.
                    addMessage(f"There may be an issue with the Melvin WCS X axis. \n    \u2022 Difference between 'Stud' and 'Exterior' bodies: {difference:.3f}in")
            else:
                offset_x = corner_x - in_cm(4.6875)

        # This tries to set the WCS Y axis to location identified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if east_return_result == adsk.core.DialogResults.DialogYes:
            if any("Track" in body.name for body in rootComp.bRepBodies):
                offset_z = track_max_point.z
                difference = abs((abs(max_z)) - (abs(track_max_point.z)) / 2.54)
                if abs(difference - 0.063) > .002:
                    addMessage(f"There may be an issue with the Melvin WCS Y axis. \n    \u2022 Difference between 'Track' and 'Exterior' bodies: {difference:.3f}in")
           
            else:
                offset_z = corner_z - in_cm(0.0625)

        construction_point = adsk.core.Point3D.create(offset_x, offset_y, offset_z)

        # Create the point in the design
        sketches = rootComp.sketches
        sketch = sketches.add(rootComp.xYConstructionPlane)
        sketchPoint = sketch.sketchPoints.add(construction_point)
        constructionPoints = rootComp.constructionPoints
        point_input = constructionPoints.createInput()
        point_input.setByPoint(sketchPoint)
        new_point = constructionPoints.add(point_input)
        new_point.name = 'Melvin WCS'
        app.activeViewport.fit()

        ############ Define Melvin 'Flipped' WCS ############
        # Back-bottom-left corner
        bl_corner_x, bl_corner_y, bl_corner_z = min_x, max_y, min_z

        # Define WCS X axis position
        if any("Stud" in body.name for body in rootComp.bRepBodies):
            bl_offset_x = stud_min_point.x
        else:
            bl_offset_x = bl_corner_x + in_cm(0.0625)

        # Define WCS Y axis position
        if any("Track" in body.name for body in rootComp.bRepBodies):
            bl_offset_z = track_min_point.z
        else:
            bl_offset_z = bl_corner_z + in_cm(0.0625)

        # Define WCS Z axis position
        bl_offset_y = bl_corner_y - in_cm(6.0)

        # This tries to set the WCS X axis to location identified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if west_return_result == adsk.core.DialogResults.DialogYes:
            if any("Stud" in body.name for body in rootComp.bRepBodies):
                bl_offset_x = stud_min_point.x
                if (abs(min_x)) - (abs(stud_min_point.x)) != in_cm(0.0625) or in_cm (4.6875): #Probably need sheathing width as well.
                    addMessage("There may be an issue with the Melvin 'Flipped' WCS X axis")
            else:
                bl_offset_x = bl_corner_x + in_cm(4.6875)

        # This tries to set the WCS Y axis to location identified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if west_return_result == adsk.core.DialogResults.DialogYes:
            if any("Track" in body.name for body in rootComp.bRepBodies):
                bl_offset_x = bl_corner_x - (abs(track_min_point.z))
                if (abs(max_z)) - (abs(track_max_point.z)) != in_cm(0.0625):
                    addMessage("There may be an issue with the Melvin 'Flipped' WCS Y axis")
            else:
                offset_z = corner_z - in_cm(0.0625)

        # Create point
        construction_point = adsk.core.Point3D.create(bl_offset_x, bl_offset_y, bl_offset_z)
        sketches = rootComp.sketches
        sketch = sketches.add(rootComp.xYConstructionPlane)
        sketchPoint = sketch.sketchPoints.add(construction_point)
        constructionPoints = rootComp.constructionPoints
        point_input = constructionPoints.createInput()
        point_input.setByPoint(sketchPoint)
        new_point = constructionPoints.add(point_input)
        new_point.name = 'Melvin Flipped WCS'
        app.activeViewport.fit()

    except:
        ui.messageBox(f"melvinOrigin(): failed:\n{traceback.format_exc()}")

def charlesOrigin():
    try:
        body = rootComp.bRepBodies.item(0)
        
        min_x, max_x, min_y, max_y, min_z, max_z = float('inf'), float('-inf'), float('inf'), float('-inf'), float('inf'), float('-inf')

        for face in body.faces:
            boundingBox = face.boundingBox
            min_x = min(min_x, boundingBox.minPoint.x); max_x = max(max_x, boundingBox.maxPoint.x)
            min_y = min(min_y, boundingBox.minPoint.y); max_y = max(max_y, boundingBox.maxPoint.y)
            min_z = min(min_z, boundingBox.minPoint.z); max_z = max(max_z, boundingBox.maxPoint.z)
        
        # Max extents define the "Back Top Right" corner
        corner_x, corner_y, corner_z = max_x, max_y, max_z
        
        # Default WCS offset if frame bodies are not identified. (0.0625 inward for X and Z, no offset for Y)
        offset_x = corner_x - in_cm(0.0625)
        offset_y = corner_y
        offset_z = corner_z - in_cm(0.0625)

        if any("Stud" in body.name for body in rootComp.bRepBodies):
            offset_x = stud_max_point.x

        if any("Track" in body.name for body in rootComp.bRepBodies):
            offset_z = track_max_point.z

        if east_return_result == adsk.core.DialogResults.DialogYes:
            if any("Stud" in body.name for body in rootComp.bRepBodies):
                offset_x = stud_max_point.x
                offset = abs(((abs(max_x)) - (abs(stud_max_point.x)))) / 2.54
                if not (offset - 0.0625) > .01 or (offset - 4.625) > .01: #Probably need to add sheathing width as well.S
                    addMessage(f"There may be an issue with the Charles WCS X axis position. \n     \u2022{offset:.3f} offset was detected.")  
            else:
                offset_x = corner_x - in_cm(4.625)

        if any("Track" in body.name for body in rootComp.bRepBodies):
                offset_z = track_max_point.z
                offset = abs(((abs(max_z)) - (abs(track_max_point.z)))) / 2.54
                if (offset - 0.0625) > .01 or (offset - 4.625) > .01: #Probably need to add sheathing width as well.
                    addMessage(f"There may be an issue with the Charles WCS Y axis position. \n     \u2022{offset:.3f} offset was detected.")
       
        construction_point = adsk.core.Point3D.create(offset_x, offset_y, offset_z)

        # Create a sketch and add a sketch point
        sketches = rootComp.sketches
        sketch = sketches.add(rootComp.xYConstructionPlane)  # temporary reference plane
        sketchPoint = sketch.sketchPoints.add(construction_point)
        constructionPoints = rootComp.constructionPoints
        point_input = rootComp.constructionPoints.createInput()
        point_input.setByPoint(sketchPoint)
        new_point = constructionPoints.add(point_input)
        new_point.name = 'Charles WCS'
        app.activeViewport.fit()
    except:
        ui.messageBox(f"charlesOrigin(): failed:\n{traceback.format_exc()}")

def camWorkspace():
    # Define global variables
    global cam, setups, doc, cam_product,camOcc, setupInput
    
    try:
        # Switch to Manufacture Workspace
        ui.workspaces.itemById('CAMEnvironment').activate()

        # Get the active product.
        cam = adsk.cam.CAM.cast(app.activeProduct)

        # Get the Setups collection.
        setups = cam.setups

        # Create a SetupsInput object to define a milling setup.
        setupInput = setups.createInput(adsk.cam.OperationTypes.MillingOperation)

        # Get the CAM product from the document's products collection
        doc = app.activeDocument
        cam_product = doc.products.itemByProductType('CAMProductType')
        cam = adsk.cam.CAM.cast(cam_product)

        # Specify the first body in the model as the model geometry.
        camOcc = cam.designRootOccurrence
        setupInput.models = [camOcc.bRepBodies[0]]
    except:
        ui.messageBox(f"camWorkspace(): failed:\n{traceback.format_exc()}")

def idBrickDetail():
    try:
        offset = in_cm(0.5)
        faces_at_offset = []

        # Step 1: Find the "Exterior" body
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":
                exterior_body = body
                break

        # Step 2: Maximum Y of Exterior body
        max_y = exterior_body.boundingBox.minPoint.y

        # Step 3: Find faces exactly offset_inch below max Y
        target_y = max_y + offset
        for face in exterior_body.faces:
            face_y = face.centroid.y
            if abs(face_y - target_y) <= .002:  # tiny tolerance for float comparison
                faces_at_offset.append(face)

        if len(faces_at_offset) > 0:
            return True
        
        else:
            return False

    except:
        ui.messageBox(f"idBrickDetail() failed:\n{traceback.format_exc()}")

def idEastReturnBrickDetail():
    try:
        offset = in_cm(0.5)
        faces_at_offset = []

        # Step 1: Find the "Exterior" body
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":
                exterior_body = body
                break

        # Step 2: Maximum Y of Exterior body
        max_x = exterior_body.boundingBox.maxPoint.x

        # Step 3: Find faces exactly offset_inch below max Y
        target_x = max_x - offset
        for face in exterior_body.faces:
            face_x = face.centroid.x
            if abs(face_x - target_x) <= .002:  # tiny tolerance for float comparison
                faces_at_offset.append(face)

        if len(faces_at_offset) > 0:
            return True
        
        else:
            return False

    except:
        ui.messageBox(f"idEastReturnBrickDetail() failed:\n{traceback.format_exc()}")

def idWestReturnBrickDetail():
    try:
        offset = in_cm(0.5)
        faces_at_offset = []

        # Step 1: Find the "Exterior" body
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":
                exterior_body = body
                break

        # Step 2: Maximum Y of Exterior body
        min_x = exterior_body.boundingBox.minPoint.x

        # Step 3: Find faces exactly offset_inch below max Y
        target_x = min_x + offset
        for face in exterior_body.faces:
            face_x = face.centroid.x
            if abs(face_x - target_x) <= .002:  # tiny tolerance for float comparison
                faces_at_offset.append(face)

        if len(faces_at_offset) > 0:
            return True
        
        else:
            return False

    except:
        ui.messageBox(f"idWestReturnBrickDetail() failed:\n{traceback.format_exc()}")

def melvinSetup():
    try:
        if not any("Sheathing" in body.name for body in rootComp.bRepBodies):
            # Show Exterior Body for Charles setup.
            for body in rootComp.bRepBodies:
                if body.name == "Exterior":
                    body.isVisible = True
            # Alert user.
            addMessage("\"Sheathing\" body was not detected.")
            addMessage("\"The Melvin setup was not created.")
            return
        else:
            # Create the setup.
            setup = setups.add(setupInput)

            # Set the name of the setup to "Melvin"
            setup.name = "Melvin"

            # Set the program name to file name + "M" for Melvin
            progNameParam = setup.parameters.itemByName('job_programName')
            stringVal: adsk.cam.StringParameterValue = progNameParam.value
            current_doc_name = app.activeDocument.name
            cleaned_name = re.sub(r'\s*[vV]\d+', '', current_doc_name) 
            stringVal.value = (cleaned_name + "M")

            # Set the comment for the program.
            commentParam = setupInput.parameters.itemByName('job_programComment')
            commentParam.value.value = 'Created from PanelStartUp.py script version 1.0'
            
            # Find and assign the machine from the Local Library
            machine_model_to_find = "Melvin"
            found_machine = None

            # Get the CAMManager and LibraryManager
            camManager = adsk.cam.CAMManager.get()
            libraryManager = camManager.libraryManager

            # Get the singular MachineLibrary object
            machineLibrary = libraryManager.machineLibrary

            # Get the URL for the Local machine library location
            local_machine_url = machineLibrary.urlByLocation(adsk.cam.LibraryLocations.LocalLibraryLocation)
            #local_machine_url = machineLibrary.urlByLocation(adsk.cam.LibraryLocations.CloudLibraryLocation)

            # Get the machines from this specific local library URL
            local_machines = list(machineLibrary.childMachines(local_machine_url))

            # Loop through the found machines to find yours by model name
            for machine_item in local_machines:
                if machine_item.model == machine_model_to_find:
                    found_machine = machine_item
                    break
                    
            setup.machine = found_machine

            # Select Stock for Melvin setup
            stock_body = rootComp.bRepBodies.itemByName('Sheathing')

            # Change stock mode
            prmStockMode = setup.parameters.itemByName('job_stockMode')
            prmStockMode.expression = "'solid'"

            stock_solids_collection = adsk.core.ObjectCollection.create()
            stock_solids_collection.add(stock_body)
            setup.stockSolids = stock_solids_collection 

            # Select origin for Melvin setup
            sketchPoint = rootComp.constructionPoints.itemByName('Melvin WCS')
            setup.parameters.itemByName('wcs_origin_mode').expression = "'point'"
            setup.parameters.itemByName('wcs_origin_point').value.value = [sketchPoint]
            setup.parameters.itemByName('wcs_orientation_flipX').value.value = True
            setup.parameters.itemByName('wcs_orientation_flipZ').value.value = True

            # Get the model's Y construction axis
            z_axis_entity = rootComp.yConstructionAxis 
            # Assign a Python list containing the axis entity directly
            setup.parameters.itemByName('wcs_orientation_axisZ').value.value = [z_axis_entity]
            
            # Load templates from cloud for Melvin
            template_names_to_load = [
                    "Melvin 2 Pass NEW"
                ]

            camManager = adsk.cam.CAMManager.get()
            libraryManager = camManager.libraryManager
            templateLibrary = libraryManager.templateLibrary

            cloud_template_url = templateLibrary.urlByLocation(adsk.cam.LibraryLocations.CloudLibraryLocation)

            cloud_templates = list(templateLibrary.childTemplates(cloud_template_url))

            for template_name in template_names_to_load:
                found_template = [item for item in cloud_templates if item.name == template_name][0] # Assumes template is found
                setup.createFromCAMTemplate(found_template)

            # Set entry point for Perimeter ToolPath
            melvin_setup = None
            for setup_index in range(cam.setups.count):
                current_setup = cam.setups.item(setup_index)
                if current_setup.name == "Melvin":
                    melvin_setup = current_setup
                    break

            perimeter_op = None
            for op_index in range(melvin_setup.operations.count):
                op = melvin_setup.operations.item(op_index)
                if op.name == "Perimeter":
                    perimeter_op = op
                    break 

            perimeter_op.parameters.itemByName('entryPositions').value.value = [sketchPoint]
    except:
        ui.messageBox(f"melvinSetup(): failed:\n{traceback.format_exc()}")

def charlesSetup():
    try:
        # Attempts to confirm there is foam on this panel by checking the thickness of the 'Exterior' body 
        # Excluding 10 inches from either side to avoid false positives from 'bumps'.
        foamResult = False
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":  
                
                # Exclude 10" from either side
                exclude = in_cm(10) 

                # Get bounding box for X limits
                bb = body.boundingBox
                minX = bb.minPoint.x + exclude
                maxX = bb.maxPoint.x - exclude

                minY = bb.minPoint.y
                maxY = bb.maxPoint.y

                # Loop through all vertices
                for v in body.vertices:
                    x = v.geometry.x
                    y = v.geometry.y

                    if x >= minX and x <= maxX:
                        if y < minY:
                            minY = y
                        if y > maxY:
                            maxY = y

                # Thickness along Y in inches
                thickness = abs(maxY - minY) / 2.54  # cm to inches

                if thickness is not None and thickness > 6.9:
                    foamResult = True

        if foamResult == True:
            # Create a SetupsInput object to define a milling setup.
            setupInput = setups.createInput(adsk.cam.OperationTypes.MillingOperation)

            # Get the CAM product from the document's products collection
            doc = app.activeDocument
            cam_product = doc.products.itemByProductType('CAMProductType')
            cam = adsk.cam.CAM.cast(cam_product)

            # Specify the first body in the model as the model geometry.
            camOcc = cam.designRootOccurrence
            setupInput.models = [camOcc.bRepBodies[0]]

            # Set the origin to be at the top center of the model box.
            originParam = setupInput.parameters.itemByName('wcs_origin_mode')
            choiceVal: adsk.cam.ChoiceParameterValue = originParam.value
            choiceVal.value = 'modelPoint'

            originPoint = setupInput.parameters.itemByName('wcs_origin_boxPoint')
            choiceVal: adsk.cam.ChoiceParameterValue = originPoint.value
            choiceVal.value = 'top center'

            # Set the comment for the program.
            commentParam = setupInput.parameters.itemByName('job_programComment')
            commentParam.value.value = 'Created from script version 1.0'

            # Create the setup.
            global setup
            setup = setups.add(setupInput)

            # Set the name of the setup to "Melvin"
            setup.name = "Charles"

            # Set the program name to file name + "M" for Melvin
            progNameParam = setup.parameters.itemByName('job_programName')
            stringVal: adsk.cam.StringParameterValue = progNameParam.value
            current_doc_name = app.activeDocument.name
            cleaned_name = re.sub(r'\s*[vV]\d+', '', current_doc_name) 
            stringVal.value = (cleaned_name + "C")

            # Find and assign the machine from the Local Library
            machine_model_to_find = "Charles"
            found_machine = None

            # Get the CAMManager and LibraryManager
            camManager = adsk.cam.CAMManager.get()
            libraryManager = camManager.libraryManager

            # Get the singular MachineLibrary object
            machineLibrary = libraryManager.machineLibrary

            # Get the URL for the Local machine library location
            local_machine_url = machineLibrary.urlByLocation(adsk.cam.LibraryLocations.CloudLibraryLocation)

            # Get the machines from this specific local library URL
            local_machines = list(machineLibrary.childMachines(local_machine_url))

            # Loop through the found machines to find yours by model name
            for machine_item in local_machines:
                if machine_item.model == machine_model_to_find:
                    found_machine = machine_item
                    break
                    
            setup.machine = found_machine

            # Select Stock for Charles setup
            stock_body = rootComp.bRepBodies.itemByName('Stock')

            # Change stock mode
            prmStockMode = setup.parameters.itemByName('job_stockMode')
            prmStockMode.expression = "'solid'"

            stock_solids_collection = adsk.core.ObjectCollection.create()
            stock_solids_collection.add(stock_body)
            setup.stockSolids = stock_solids_collection 

            # Set Origin
            sketchPoint = rootComp.constructionPoints.itemByName('Charles WCS')
            setup.parameters.itemByName('wcs_origin_mode').expression = "'point'"
            setup.parameters.itemByName('wcs_origin_point').value.value = [sketchPoint]
            setup.parameters.itemByName('wcs_orientation_flipX').value.value = True
            setup.parameters.itemByName('wcs_orientation_flipZ').value.value = True

            # Get the model's Y construction axis
            z_axis_entity = rootComp.yConstructionAxis 
            # Assign a Python list containing the axis entity directly
            setup.parameters.itemByName('wcs_orientation_axisZ').value.value = [z_axis_entity]
            
            template_names_to_load = []
            # Load templates from cloud for Charles by tool number
            template_names_to_load.extend([
                    "Charles Facinghead",
                    "Charles Perimeter",                                    # Tool 1
                    "Charles Perimeter Above Sheathing"                     # Tool 1
            ])

            if windows == True:
                template_names_to_load.append("Charles Window Bevel")       # Tool 1

            if east_return_result == adsk.core.DialogResults.DialogYes:
                # Load templates from cloud for Charles
                template_names_to_load.extend(["Charles Return EM"])        # Tool 1
            
            if west_return_result == adsk.core.DialogResults.DialogYes:
                template_names_to_load.extend(["Charles Return EM" ])       # Tool 1

            if idBrickDetail():
                template_names_to_load.extend([
                        "Charles Brick Feature EM",                         # Tool 2
                        "Charles Brick Feature FM"                          # Tool 3
                    ])
                
            # if any("Bump" in body.name for body in rootComp.bRepBodies):
            if eastBump == True:
                template_names_to_load.append("Charles Bump Clean Up FM")   # Tool 3   

            if east_return_result == adsk.core.DialogResults.DialogYes:
                # Load templates from cloud for Charles
                template_names_to_load.extend(["Charles Return FM"])        # Tool 3
                
            if west_return_result == adsk.core.DialogResults.DialogYes:
                template_names_to_load.extend(["Charles Return FM"])        # Tool 3
                
            if idWestReturnBrickDetail():
                template_names_to_load.extend(["Charles Return Brick FM"])  # Tool 3
        
            if idEastReturnBrickDetail():
                template_names_to_load.extend(["Charles Return Brick FM"])  # Tool 3
        
            camManager = adsk.cam.CAMManager.get()
            libraryManager = camManager.libraryManager
            templateLibrary = libraryManager.templateLibrary

            cloud_template_url = templateLibrary.urlByLocation(adsk.cam.LibraryLocations.CloudLibraryLocation)

            cloud_templates = list(templateLibrary.childTemplates(cloud_template_url))

            for template_name in template_names_to_load:
                found_template = [item for item in cloud_templates if item.name == template_name][0] 
                setup.createFromCAMTemplate(found_template)

            if east_return_result == adsk.core.DialogResults.DialogYes:
                changeEastReturnEM = setup.operations.itemByName('Return EM')
                changeEastReturnEM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]
                changeEastReturnFM = setup.operations.itemByName('Return FM')
                changeEastReturnFM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]

            if west_return_result == adsk.core.DialogResults.DialogYes:
                changeWestReturnEM = setup.operations.itemByName('Return EM')
                changeWestReturnEM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]
                changeWestReturnEM.parameters.itemByName("view_orientation_flipZ").value.value = True
                changeWestReturnFM = setup.operations.itemByName('Return FM')
                changeWestReturnFM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]
                changeWestReturnFM.parameters.itemByName("view_orientation_flipZ").value.value = True
                 
            if idEastReturnBrickDetail():
                eastReturnBrick = setup.operations.itemByName('Return Brick FM')
                eastReturnBrick.parameters.itemByName('view_orientation_axisZ').value.value = [rootComp.yZConstructionPlane]

            if idWestReturnBrickDetail():
                westReturnBrick = setup.operations.itemByName('Return Brick FM')
                westReturnBrick.parameters.itemByName('view_orientation_axisZ').value.value = [rootComp.yZConstructionPlane]
                westReturnBrick.parameters.itemByName("view_orientation_flipZ").value.value = True

        else:
            addMessage("'Foam' body was not detected. The Charles setup will not be created.")

        if any("Bump" in body.name for body in rootComp.bRepBodies):
            bumpMod()

        if westBump == True or eastBump == True:
            bumpMod()

    except:
        ui.messageBox(f"{traceback.format_exc()}", "charlesSetup(): failed!")

def bumpMod():                  # This function modifies the Facinghead toolpath if a bump is detected
    try:
        if 'Charles' in [setup.name for setup in setups]:
            for body in rootComp.bRepBodies:
                if body.name.startswith("Bump"):
                    body.isVisible = True

            if eastBump == True:
                # Modify 'Facinghead' toolpath
                facinghead_input = setup.operations.itemByName('Facinghead')
                facinghead_input.parameters.itemByName('passAngle').expression = '0 deg'
                facinghead_input.parameters.itemByName('transitionType').expression = "'straight-line'"
                cam.generateToolpath(facinghead_input)
                
            elif westBump == True:
                # Modify 'Facinghead' toolpath
                facinghead_input = setup.operations.itemByName('Facinghead')
                facinghead_input.parameters.itemByName('passAngle').expression = '180 deg'
                facinghead_input.parameters.itemByName('transitionType').expression = "'straight-line'"
                cam.generateToolpath(facinghead_input) 
                
            else:
                None
    except:
        ui.messageBox(f"bumpMod(): failed:\n{traceback.format_exc()}")

def idThinFoam():
    try:
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":  
                boundingBox = body.boundingBox
                width =  (abs(boundingBox.minPoint.y) - abs(boundingBox.maxPoint.y)) / 2.54
                    
        if width > 6.9 and width < 9.25:
            facinghead_input = setup.operations.itemByName('Facinghead')
            facinghead_input.parameters.itemByName('bottomHeight_offset').expression = '8.25 in'
            facinghead_input.parameters.itemByName('topHeight_offset').expression = '8.5 in'
            cam.generateToolpath(facinghead_input)

            if setup.operations.itemByName('Brick Feature EM'):
                brick_em_input = setup.operations.itemByName('Brick Feature EM')
                brick_em_input.parameters.itemByName('bottomHeight_offset').expression = '7.75 in'
                cam.generateToolpath(brick_em_input)

            if setup.operations.itemByName('Brick Feature FM'):
                brick_fm_input = setup.operations.itemByName('Brick Feature FM')
                brick_fm_input.parameters.itemByName('bottomHeight_offset').expression = '7.75 in'
                cam.generateToolpath(brick_fm_input)
            
    except:
        ui.messageBox(f"thinFoam(): failed:\n{traceback.format_exc()}")

def melvinGeometry():
    try:
        
        # ---- CONFIG ----
        BODY_NAME = "Sheathing"
        SETUP_NAME = "Melvin"
        OP_NAME = "Perimeter"

        # Outer chain Y range
        Y_MIN_IN = -6.21
        Y_MAX_IN = -6.0
        Y_TOL = 0.01
        COORD_TOL = 1e-3

        # Inner loops Y target
        INNER_Y_IN = -6.0
        INNER_Y = in_cm(INNER_Y_IN)

        Y_MIN = in_cm(Y_MIN_IN)
        Y_MAX = in_cm(Y_MAX_IN)

        # ---- FIND BODY ----
        body = None
        for b in rootComp.bRepBodies:
            if b.name == BODY_NAME:
                body = b
                break
        if not body:
            #ui.messageBox(f"Body '{BODY_NAME}' not found.")
            return

        # ---- GET EXTREMES ----
        xmin = min(v.geometry.x for v in body.vertices)
        xmax = max(v.geometry.x for v in body.vertices)
        zmin = min(v.geometry.z for v in body.vertices)
        zmax = max(v.geometry.z for v in body.vertices)

        def on_extreme_plane(p):
            return (
                abs(p.x - xmin) < COORD_TOL or
                abs(p.x - xmax) < COORD_TOL or
                abs(p.z - zmin) < COORD_TOL or
                abs(p.z - zmax) < COORD_TOL
            )

        # ---- FIND OUTER LOOP EDGES ----
        loop_edges = []
        seen = set()
        for face in body.faces:
            for edge in face.edges:
                p1 = edge.startVertex.geometry
                p2 = edge.endVertex.geometry
                key = tuple(
                    sorted([
                        (round(p1.x, 4), round(p1.y, 4), round(p1.z, 4)),
                        (round(p2.x, 4), round(p2.y, 4), round(p2.z, 4))
                    ])
                )
                if key in seen:
                    continue
                seen.add(key)

                # within Y range
                if not ((Y_MIN - Y_TOL) <= p1.y <= (Y_MAX + Y_TOL) and
                        (Y_MIN - Y_TOL) <= p2.y <= (Y_MAX + Y_TOL)):
                    continue

                # both vertices must be on min/max X/Z planes
                if on_extreme_plane(p1) and on_extreme_plane(p2):

                    # ---- APPLY BUMP LOGIC ----
                    if westBump and abs(p1.x - xmin) < COORD_TOL and abs(p2.x - xmin) < COORD_TOL:
                        continue  # skip forbidden west edge
                    if eastBump and abs(p1.x - xmax) < COORD_TOL and abs(p2.x - xmax) < COORD_TOL:
                        continue  # skip forbidden east edge

                    loop_edges.append(edge)

        if not loop_edges:
            ui.messageBox("No outer perimeter edges found.")
            return

        # ---- ORDER INTO LOOP ----
        def order_edges(edges):
            vert_map = {}
            for e in edges:
                v1, v2 = e.startVertex, e.endVertex
                vert_map.setdefault(v1.tempId, []).append(e)
                vert_map.setdefault(v2.tempId, []).append(e)

            ordered = []
            visited = set()
            current = edges[0]
            ordered.append(current)
            visited.add(current.tempId)
            current_vert = current.endVertex

            while len(ordered) < len(edges):
                next_edges = vert_map.get(current_vert.tempId, [])
                next_edge = None
                for e in next_edges:
                    if e.tempId not in visited:
                        next_edge = e
                        break
                if not next_edge:
                    break
                ordered.append(next_edge)
                visited.add(next_edge.tempId)
                if next_edge.startVertex.tempId == current_vert.tempId:
                    current_vert = next_edge.endVertex
                else:
                    current_vert = next_edge.startVertex
            return ordered

        outer_loop = order_edges(loop_edges)

        # ---- FIND INNER LOOPS (Y = -6.0 in, NOT on X/Z extremes) ----
        inner_edges = []
        for face in body.faces:
            for edge in face.edges:
                p1 = edge.startVertex.geometry
                p2 = edge.endVertex.geometry
                if abs(p1.y - INNER_Y) < Y_TOL and abs(p2.y - INNER_Y) < Y_TOL:
                    # reject if any vertex is on an extreme plane
                    if not (on_extreme_plane(p1) or on_extreme_plane(p2)):
                        inner_edges.append(edge)

        # ---- GROUP INTO CLOSED LOOPS ----
        loops = []
        used = set()
        for e in inner_edges:
            if e.tempId in used:
                continue
            loop = [e]
            used.add(e.tempId)
            vmap = {e.startVertex.tempId: e, e.endVertex.tempId: e}
            changed = True
            while changed:
                changed = False
                for e2 in inner_edges:
                    if e2.tempId in used:
                        continue
                    for v in [e2.startVertex, e2.endVertex]:
                        if v.tempId in vmap:
                            loop.append(e2)
                            used.add(e2.tempId)
                            vmap[e2.startVertex.tempId] = e2
                            vmap[e2.endVertex.tempId] = e2
                            changed = True
                            break
            if len(loop) > 2:
                loops.append(order_edges(loop))

        # ---- CAM ENV ----
        ui.workspaces.itemById("CAMEnvironment").activate()
        cam = adsk.cam.CAM.cast(app.activeProduct)
        setup = cam.setups.itemByName(SETUP_NAME)
        if not setup:
            ui.messageBox(f"Setup '{SETUP_NAME}' not found.")
            return
        op = setup.operations.itemByName(OP_NAME)
        if not op:
            ui.messageBox(f"Operation '{OP_NAME}' not found.")
            return

        # ---- FIND CHAIN PARAM ----
        contourParam = None
        for pname in ["curves"]:
            param = op.parameters.itemByName(pname)
            if param:
                try:
                    contourParam = param.value
                    break
                except:
                    pass
        if not contourParam:
            for attr in ["contourParameters", "chainCurves"]:
                try:
                    contourParam = getattr(op, attr)
                    if contourParam:
                        break
                except:
                    pass
        if not contourParam:
            ui.messageBox(f"Could not find chain parameter for '{OP_NAME}'.")
            return

        curveSelections = contourParam.getCurveSelections()
        try:
            curveSelections.clear()
        except:
            pass

        # ---- ADD OUTER LOOP ----
        outer_chain = curveSelections.createNewChainSelection()
        # Make it OPEN if eastBump or westBump
        outer_chain.isOpen = False
        if westBump == True or eastBump == True:
            outer_chain.isOpen = True
        outer_chain.isReverted = True
        outer_chain.inputGeometry = outer_loop

        # ---- ADD INNER LOOPS ----
        for inner in loops:
            ch = curveSelections.createNewChainSelection()
            ch.isOpen = False
            ch.reverse = False
            ch.inputGeometry = inner

        contourParam.applyCurveSelections(curveSelections)

        cam.generateToolpath(op) 

    except:
        ui.messageBox(f"melvinGeometry(): failed:\n{traceback.format_exc()}")

def facingheadGeometry():
    try:
        # --- 1. Find Stock Body ---
        stock_body = None
        for body in rootComp.bRepBodies:
            if body.name == 'Stock':
                stock_body = body
                break
        if not stock_body:
            ui.messageBox("No 'Stock' body found.")
            return

        # --- 2. Find ALL Bottom Flat Faces ---
        flat_faces = []
        for face in stock_body.faces:
            bb = face.boundingBox
            # Check for flat face (Y-constant plane)
            if abs(bb.maxPoint.y - bb.minPoint.y) < 1e-6:
                flat_faces.append(face)

        if not flat_faces:
            ui.messageBox("No flat faces found on the Stock body.")
            return

        # Find the lowest Y value among all flat faces
        ymin = min(f.boundingBox.minPoint.y for f in flat_faces)

        # Tolerance for coplanarity
        PLANE_TOL = 1e-4

        # Collect all faces whose minY ≈ ymin (bottom plane)
        bottom_faces = [
            f for f in flat_faces
            if abs(f.boundingBox.minPoint.y - ymin) < PLANE_TOL
        ]

        if not bottom_faces:
            ui.messageBox("No bottom faces found at min Y.")
            return

        # --- 3. Identify Faces 0.5" Above the Bottom Plane ---
        offset = in_cm(0.5)
        target_y = ymin + offset
        TOL = 0.001

        target_faces = list(bottom_faces)  # include bottom layer

        for face in flat_faces:
            y = face.boundingBox.minPoint.y
            if abs(y - target_y) < TOL:
                target_faces.append(face)

        # --- 4. Apply to Toolpath ---
        setup = cam.setups.itemByName('Charles')
        op = setup.operations.itemByName('Facinghead')

        contourParam: adsk.cam.CadContours2dParameterValue = op.parameters.itemByName('stockContours').value
        curveSelections = contourParam.getCurveSelections()
        curveSelections.clear()

        new_selection_list = adsk.core.ObjectCollection.create()

        for face in target_faces:
            fc: adsk.cam.FaceContourSelection = curveSelections.createNewFaceContourSelection()
            fc.loopType = adsk.cam.LoopTypes.AllLoops
            fc.sideType = adsk.cam.SideTypes.StartOutsideSideType
            fc.inputGeometry = [face]
            new_selection_list.add(fc)

        curveSelections.curveSelections = new_selection_list
        contourParam.applyCurveSelections(curveSelections)

        cam.generateToolpath(op)

        #ui.messageBox(f"Selected {len(target_faces)} faces (bottom + 0.5in).")

    except:
        ui.messageBox(f"facingheadGeometry(): failed:\n{traceback.format_exc()}")

def perimeterGeometry():
    try:
        # Find 'Exterior' body
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == 'Exterior':
                exterior_body = body
                break
        if not exterior_body:
            ui.messageBox("Error: Could not find a body named 'Exterior'.")
            return

        # Bounding box and extremes
        bbox = exterior_body.boundingBox
        xmin, xmax = bbox.minPoint.x, bbox.maxPoint.x
        zmin, zmax = bbox.minPoint.z, bbox.maxPoint.z
        min_y = bbox.minPoint.y
        half_inch = in_cm(0.5)
        target_y_levels = (min_y, min_y + half_inch)
        Y_TOL = 0.002
        COORD_TOL = 1e-4

        # --- Helpers ---
        def vgeom(v):
            return v.geometry

        def vertex_key(v):
            p = vgeom(v)
            return (round(p.x,6), round(p.y,6), round(p.z,6))

        def edge_key(e):
            v1 = vertex_key(e.startVertex)
            v2 = vertex_key(e.endVertex)
            return tuple(sorted([v1,v2]))

        def is_vertex_on_y(v):
            p = vgeom(v)
            return any(abs(p.y - ty) < Y_TOL for ty in target_y_levels)

        def edge_on_exterior_plane(edge):
            p1 = edge.startVertex.geometry
            p2 = edge.endVertex.geometry
            # Both vertices must be on the SAME extreme plane
            if abs(p1.x - xmin) < COORD_TOL and abs(p2.x - xmin) < COORD_TOL:
                return True
            if abs(p1.x - xmax) < COORD_TOL and abs(p2.x - xmax) < COORD_TOL:
                return True
            if abs(p1.z - zmin) < COORD_TOL and abs(p2.z - zmin) < COORD_TOL:
                return True
            if abs(p1.z - zmax) < COORD_TOL and abs(p2.z - zmax) < COORD_TOL:
                return True
            return False

        # --- Collect perimeter edges strictly ---
        perimeter_edges = []
        for face in exterior_body.faces:
            for edge in face.edges:
                if is_vertex_on_y(edge.startVertex) and is_vertex_on_y(edge.endVertex):
                    if edge_on_exterior_plane(edge):
                        p1, p2 = edge.startVertex.geometry, edge.endVertex.geometry
                        # Apply eastBump / westBump filters
                        if eastBump == True and abs(p1.x - xmax) < COORD_TOL and abs(p2.x - xmax) < COORD_TOL:
                            continue  # skip edges on xmax if eastBump is True
                        if westBump == True and abs(p1.x - xmin) < COORD_TOL and abs(p2.x - xmin) < COORD_TOL:
                            continue  # skip edges on xmin if westBump is True
                        perimeter_edges.append(edge)

        if not perimeter_edges:
            ui.messageBox("No perimeter edges found on target Y levels.")
            return

        # --- Group connected edges into chains ---
        vertex_to_edges = {}
        for e in perimeter_edges:
            v1, v2 = vertex_key(e.startVertex), vertex_key(e.endVertex)
            vertex_to_edges.setdefault(v1, []).append(e)
            vertex_to_edges.setdefault(v2, []).append(e)

        visited = set()
        chains = []
        for e in perimeter_edges:
            ek = edge_key(e)
            if ek in visited:
                continue
            stack = [e]
            chain_edges = []
            while stack:
                cur = stack.pop()
                cur_key = edge_key(cur)
                if cur_key in visited:
                    continue
                visited.add(cur_key)
                chain_edges.append(cur)
                v1, v2 = vertex_key(cur.startVertex), vertex_key(cur.endVertex)
                for nxt in vertex_to_edges.get(v1, []) + vertex_to_edges.get(v2, []):
                    if edge_key(nxt) not in visited:
                        stack.append(nxt)
            if chain_edges:
                chains.append(chain_edges)

        # --- Compute centroid and sort clockwise ---
        all_x, all_z = [], []
        for e in perimeter_edges:
            for v in [e.startVertex, e.endVertex]:
                p = vgeom(v)
                all_x.append(p.x)
                all_z.append(p.z)
        xmid, zmid = (min(all_x) + max(all_x))/2.0, (min(all_z) + max(all_z))/2.0
        start_x, start_z = max(all_x), min(all_z)
        start_angle = math.atan2(start_z - zmid, start_x - xmid)

        def chain_centroid(chain_edges):
            seen = set()
            xs, zs = [], []
            for e in chain_edges:
                for v in [e.startVertex, e.endVertex]:
                    vk = vertex_key(v)
                    if vk in seen: continue
                    seen.add(vk)
                    p = vgeom(v)
                    xs.append(p.x)
                    zs.append(p.z)
            return (sum(xs)/len(xs), sum(zs)/len(zs)) if xs else (0.0,0.0)

        chain_infos = []
        for chain_edges in chains:
            cx, cz = chain_centroid(chain_edges)
            angle = math.atan2(cz - zmid, cx - xmid)
            delta = (start_angle - angle + 2*math.pi) % (2*math.pi)
            chain_infos.append((delta, chain_edges))
        chain_infos.sort(key=lambda t: t[0])

        # --- Apply to both EM and FM operations ---
        ui.workspaces.itemById('CAMEnvironment').activate()
        cam = adsk.cam.CAM.cast(app.activeProduct)
        setup = cam.setups.itemByName('Charles')
        if not setup:
            ui.messageBox("Error: Setup 'Charles' not found.")
            return

        for op_name in ['Perimeter Ballnose EM']:
            op = None
            for operation in setup.operations:
                if operation.name == op_name:
                    op = operation
                    break

            try:
                contourParam = op.parameters.itemByName('contours').value
            except:
                contourParam = op.contourParameters  # fallback for operations where 'contours' param is missing
            curveSelections = contourParam.getCurveSelections()

            for i, (delta, chain_edges) in enumerate(chain_infos):
                chain = curveSelections.createNewChainSelection()
                try: chain.isOpenAllowed = True
                except: pass
                chain.isOpen = True
                chain.isReverted = False
                #chain.reverse = False
                chain.startExtensionLength = in_mm(5.0)
                chain.endExtensionLength = in_mm(5.0)       
                chain.inputGeometry = chain_edges  

                # if eastBump == True or westBump == True:
                #     chain.startExtensionLength = in_cm(-85.0)
                #     chain.endExtensionLength = in_cm(-85.0) 

            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)
            
    except:
        ui.messageBox(f"perimeterGeometry(): failed:\n{traceback.format_exc()}")

def aboveSheathingGeometry():
    try:
                # ---- CONFIG ----
        TARGET_Y_IN = -6.625
        Y_TOL = 0.002
        Y_MIN_IN = 1.0
        Y_MAX_IN = 5.0
        setup_name = 'Charles'
        op_name = 'Perimeter Above Sheathing'

        # ---- CONVERT UNITS ----
        TARGET_Y = in_cm(TARGET_Y_IN)
        Y_MIN = in_cm(Y_MIN_IN)
        Y_MAX = in_cm(Y_MAX_IN)

        # ---- FIND "Exterior" BODY ----
        exterior_body = None
        for b in rootComp.bRepBodies:
            if b.name == 'Exterior':
                exterior_body = b
                break
        if not exterior_body:
            ui.messageBox("Body 'Exterior' not found.")
            return

        # ---- FIND FACES WITH Y LENGTH BETWEEN MIN/MAX ----
        target_faces = []
        for face in exterior_body.faces:
            bbox = face.boundingBox
            y_length = bbox.maxPoint.y - bbox.minPoint.y
            if Y_MIN <= y_length <= Y_MAX:
                target_faces.append(face)

        if not target_faces:
            ui.messageBox(f"No faces found with Y length between {Y_MIN_IN}-{Y_MAX_IN} in.")
            return

        # ---- GET CAM SETUP AND OPERATION ----
        doc = app.activeDocument
        cam_product = doc.products.itemByProductType('CAMProductType')
        cam = adsk.cam.CAM.cast(cam_product)
        ui.workspaces.itemById("CAMEnvironment").activate()

        setup = cam.setups.itemByName(setup_name)
        if not setup:
            ui.messageBox(f"Setup '{setup_name}' not found.")
            return

        op = setup.operations.itemByName(op_name)
        if not op:
            ui.messageBox(f"Operation '{op_name}' not found.")
            return

        # ---- GET CHAIN PARAMETER ----
        contourParam = None
        for pname in ['contours', 'curves']:
            param = op.parameters.itemByName(pname)
            if param:
                try:
                    contourParam = param.value
                    break
                except:
                    pass
        if not contourParam:
            for attr in ['contourParameters', 'chainCurves']:
                try:
                    contourParam = getattr(op, attr)
                    if contourParam:
                        break
                except:
                    pass
        if not contourParam:
            ui.messageBox(f"Could not find chain parameter for '{op_name}'.")
            return

        try:
            curveSelections = contourParam.getCurveSelections()
        except:
            curveSelections = contourParam

        curveSelections.clear()

        # ---- HELPER: ORDER EDGES INTO CHAINS ----
        def create_connected_chains(edges):
            # Map vertex ID to edges
            vert_map = {}
            for e in edges:
                for v in [e.startVertex, e.endVertex]:
                    vert_map.setdefault(v.tempId, []).append(e)

            chains = []
            visited = set()

            for e in edges:
                if e.tempId in visited:
                    continue
                chain_edges = [e]
                visited.add(e.tempId)
                verts_to_check = [e.startVertex, e.endVertex]

                while verts_to_check:
                    vert = verts_to_check.pop()
                    for e2 in vert_map.get(vert.tempId, []):
                        if e2.tempId not in visited:
                            chain_edges.append(e2)
                            visited.add(e2.tempId)
                            verts_to_check.append(e2.startVertex)
                            verts_to_check.append(e2.endVertex)
                chains.append(chain_edges)
            return chains

        # ---- FIND EDGES AT TARGET Y ----
        all_edges = []
        for face in target_faces:
            for edge in face.edges:
                p1 = edge.startVertex.geometry
                p2 = edge.endVertex.geometry
                if abs(p1.y - TARGET_Y) < Y_TOL and abs(p2.y - TARGET_Y) < Y_TOL:
                    all_edges.append(edge)

        if not all_edges:
            op.deleteMe()
            addMessage("'Perimeter Above Sheathing' was removed.")
            #ui.messageBox("No edges found at target Y.")
            return

        # ---- CREATE CHAINS OF CONNECTED EDGES ----
        connected_chains = create_connected_chains(all_edges)
        total_chains = 0
        for chain_edges in connected_chains:
            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.reverse = False
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)  
            chain.inputGeometry = chain_edges
            total_chains += 1

        contourParam.applyCurveSelections(curveSelections)
        cam.generateToolpath(op)  
            
    except:
        ui.messageBox(f"aboveSheathingGeometry(): failed:\n{traceback.format_exc()}")

def windowBevelGeometry():
    try:
        if windows == True:
            TARGET_Y = in_cm(-6.625)
            Y_TOL = 0.002
            COORD_TOL = 1e-4

            # --- FIND 'Exterior' BODY ---
            exterior_body = None
            for body in rootComp.bRepBodies:
                if body.name == 'Exterior':
                    exterior_body = body
                    break
            bbox = exterior_body.boundingBox

            # --- FILTER UNIQUE EDGES PARALLEL TO X AT TARGET Y ---
            seen = set()
            parallel_edges = []

            for face in exterior_body.faces:
                for edge in face.edges:
                    p1 = edge.startVertex.geometry
                    p2 = edge.endVertex.geometry

                    key = tuple(
                        sorted([
                            (round(p1.x, 5), round(p1.y, 5), round(p1.z, 5)),
                            (round(p2.x, 5), round(p2.y, 5), round(p2.z, 5))
                        ])
                    )

                    if key in seen:
                        continue
                    seen.add(key)

                    # Both vertices near target Y
                    if abs(p1.y - TARGET_Y) < Y_TOL and abs(p2.y - TARGET_Y) < Y_TOL:
                        # Edge is parallel to X (flat in Z)
                        if abs(p1.z - p2.z) < COORD_TOL:
                            parallel_edges.append(edge)

            # --- FIND LOWEST Z VALUE ---
            lowest_z = min(edge.startVertex.geometry.z for edge in parallel_edges)
            lowest_edges = [e for e in parallel_edges if abs(e.startVertex.geometry.z - lowest_z) < COORD_TOL]

            # --- SORT BY X POSITION (start of each edge) ---
            def edge_min_x(edge):
                p1 = edge.startVertex.geometry
                p2 = edge.endVertex.geometry
                return min(p1.x, p2.x)

            lowest_edges.sort(key=edge_min_x)

            # --- PREP CAM OP ---
            ui.workspaces.itemById('CAMEnvironment').activate()
            cam = adsk.cam.CAM.cast(app.activeProduct)
            setup = cam.setups.itemByName('Charles')
            op = setup.operations.itemByName('Window Bevel')

            # --- GET TRACE PARAMETER 'curves' ---
            param = op.parameters.itemByName('curves')
            contourParam = param.value
            curveSelections = contourParam.getCurveSelections()

            # --- CLEAR AND REBUILD SELECTIONS (X-min → X-max) ---
            curveSelections.clear()

            for edge in lowest_edges:
                chain = curveSelections.createNewChainSelection()
                chain.isOpen = True
                chain.reverse = False
                chain.startExtensionLength = in_mm(-.5)
                chain.endExtensionLength = in_mm(-.5)
                if edge.startVertex.geometry.x == bbox.minPoint.x:
                    chain.startExtensionLength = in_mm(5)
                if edge.endVertex.geometry.x == bbox.maxPoint.x:
                    chain.endExtensionLength = in_mm(5)
                chain.inputGeometry = [edge]

            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)

    except:
        ui.messageBox(f"windowBevelGeometry(): failed:\n{traceback.format_exc()}")

def brickFeatureEMGeometry():
    try:
        if not idBrickDetail():
            return

        # --- Find Exterior body ---
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == 'Exterior':
                exterior_body = body
                break
        if not exterior_body:
            ui.messageBox("Error: 'Exterior' body not found.")
            return

        # --- Collect bottom planar faces (thin in Y) ---
        flat_faces = []
        for face in exterior_body.faces:
            bb = face.boundingBox
            if abs(bb.maxPoint.y - bb.minPoint.y) < 0.002:
                flat_faces.append(face)
        if not flat_faces:
            ui.messageBox("Error: No flat faces found on 'Exterior'.")
            return

        # --- Lowest Y bottom faces ---
        TOLERANCE = 0.002
        lowest_y = min(f.boundingBox.minPoint.y for f in flat_faces)
        bottom_faces = [f for f in flat_faces if abs(f.boundingBox.minPoint.y - lowest_y) < TOLERANCE]
        if not bottom_faces:
            ui.messageBox("Error: No bottom faces at the lowest Y level.")
            return

        # --- CAM setup ---
        ui.workspaces.itemById('CAMEnvironment').activate()
        cam = adsk.cam.CAM.cast(app.activeProduct)
        setup = cam.setups.itemByName('Charles')
        if not setup:
            ui.messageBox("Error: Setup 'Charles' not found.")
            return

        # Utility helpers
        def vertex_key(v):
            p = v.geometry
            return (round(p.x, 6), round(p.y, 6), round(p.z, 6))
        def edge_key(e):
            v1 = vertex_key(e.startVertex)
            v2 = vertex_key(e.endVertex)
            return tuple(sorted([v1, v2]))
        def chain_centroid(chain_edges):
            seen = set()
            xs, zs = [], []
            for e in chain_edges:
                for v in [e.startVertex, e.endVertex]:
                    vk = vertex_key(v)
                    if vk in seen:
                        continue
                    seen.add(vk)
                    p = v.geometry
                    xs.append(p.x)
                    zs.append(p.z)
            return (sum(xs) / len(xs), sum(zs) / len(zs)) if xs else (0.0, 0.0)

        all_chain_infos = []
        CONNECT_DIST = in_cm(0.51)  # 0.51" tolerance
        import math
        def dist(p1, p2):
            dx = p1.x - p2.x
            dy = p1.y - p2.y
            dz = p1.z - p2.z
            return math.sqrt(dx*dx + dy*dy + dz*dz)

        # --- For each bottom face, find matching target faces 0.5" above and collect matching edges ---
        for bottom_face in bottom_faces:
            ref_y = bottom_face.boundingBox.minPoint.y
            target_y = ref_y + in_cm(0.5)

            # collect faces at target_y that are planar thin in Y
            target_faces = []
            for f in exterior_body.faces:
                bb = f.boundingBox
                face_y = bb.minPoint.y
                if abs(bb.maxPoint.y - bb.minPoint.y) < 0.002 and abs(face_y - target_y) < TOLERANCE:
                    target_faces.append(f)
            if not target_faces:
                # skip if no target faces for this bottom_face
                continue

            # bottom edge endpoint data for tolerance matching
            bottom_edges_info = []
            for edge in bottom_face.edges:
                v1 = edge.startVertex.geometry
                v2 = edge.endVertex.geometry
                bottom_edges_info.append(((v1.x, v1.y, v1.z), (v2.x, v2.y, v2.z), edge))

            # find target edges that match bottom edges (XZ tolerance)
            target_edges = []
            for face in target_faces:
                for edge in face.edges:
                    v1 = edge.startVertex.geometry
                    v2 = edge.endVertex.geometry
                    if abs(v1.y - target_y) > TOLERANCE or abs(v2.y - target_y) > TOLERANCE:
                        continue
                    matched = False
                    for b_v1_tup, b_v2_tup, b_edge in bottom_edges_info:
                        class _P: pass
                        bp1 = _P(); bp1.x, bp1.y, bp1.z = b_v1_tup
                        bp2 = _P(); bp2.x, bp2.y, bp2.z = b_v2_tup
                        d00 = math.hypot(v1.x - bp1.x, v1.z - bp1.z)
                        d01 = math.hypot(v1.x - bp2.x, v1.z - bp2.z)
                        d10 = math.hypot(v2.x - bp1.x, v2.z - bp1.z)
                        d11 = math.hypot(v2.x - bp2.x, v2.z - bp2.z)
                        if ((d00 < CONNECT_DIST and d11 < CONNECT_DIST) or (d01 < CONNECT_DIST and d10 < CONNECT_DIST)):
                            matched = True
                            break
                    if matched:
                        target_edges.append(edge)

            if not target_edges:
                continue

            # group target_edges into connected chains using CONNECT_DIST
            visited = set()
            chains = []
            for e in target_edges:
                ek = edge_key(e)
                if ek in visited:
                    continue
                stack = [e]
                chain_edges = []
                while stack:
                    cur = stack.pop()
                    cur_key = edge_key(cur)
                    if cur_key in visited:
                        continue
                    visited.add(cur_key)
                    chain_edges.append(cur)
                    cur_v1 = cur.startVertex.geometry
                    cur_v2 = cur.endVertex.geometry
                    for nxt in target_edges:
                        nk = edge_key(nxt)
                        if nk in visited:
                            continue
                        n_v1 = nxt.startVertex.geometry
                        n_v2 = nxt.endVertex.geometry
                        if (dist(cur_v1, n_v1) < CONNECT_DIST or dist(cur_v1, n_v2) < CONNECT_DIST or
                            dist(cur_v2, n_v1) < CONNECT_DIST or dist(cur_v2, n_v2) < CONNECT_DIST):
                            stack.append(nxt)
                if chain_edges:
                    chains.append(chain_edges)

            # compute centroid and sort chains clockwise for this bottom-face group
            all_x, all_z = [], []
            for e in target_edges:
                for v in [e.startVertex, e.endVertex]:
                    p = v.geometry
                    all_x.append(p.x)
                    all_z.append(p.z)
            if not all_x:
                continue
            xmin, xmax = min(all_x), max(all_x)
            zmin, zmax = min(all_z), max(all_z)
            xmid = (xmin + xmax) / 2.0
            zmid = (zmin + zmax) / 2.0
            start_x = xmax
            start_z = zmin
            start_angle = math.atan2(start_z - zmid, start_x - xmid)

            local_chain_infos = []
            for chain_edges in chains:
                cx, cz = chain_centroid(chain_edges)
                angle = math.atan2(cz - zmid, cx - xmid)
                delta = (start_angle - angle + 2.0 * math.pi) % (2.0 * math.pi)
                local_chain_infos.append((delta, chain_edges, cx, cz, angle))
            local_chain_infos.sort(key=lambda t: t[0])
            all_chain_infos.extend(local_chain_infos)

        # --- APPLY ONLY TO 'Brick Feature EM' ---
        op = setup.operations.itemByName('Brick Feature EM')
        if not op:
            ui.messageBox("Operation 'Brick Feature EM' not found.")
            return

        try:
            contourParam = op.parameters.itemByName('contours').value
        except:
            ui.messageBox("Operation 'Brick Feature EM' does not have 'contours' parameter.")
            return

        curveSelections = contourParam.getCurveSelections()
        curveSelections.clear()

        added_chains = 0
        # apply only chains that have 2 or more edges
        for _, chain_edges, _, _, _ in all_chain_infos:
            if len(chain_edges) < 2:
                continue
            chain = curveSelections.createNewChainSelection()
            try:
                chain.isOpenAllowed = True
            except:
                pass
            chain.isOpen = True
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)
            # assign python list of edges (API expects a std::vector equivalent)
            chain.inputGeometry = [e for e in chain_edges]
            added_chains += 1

        if added_chains == 0:
            # delete the operation if no valid chains were added
            op.deleteMe()
            #ui.messageBox("No valid chains (2+ edges). 'Brick Feature EM' operation deleted.")
            return

        contourParam.applyCurveSelections(curveSelections)
        cam.generateToolpath(op)

    except:
        ui.messageBox(f"brickFeatureEMGeometry(): failed:\n{traceback.format_exc()}")

def brickFeatureFMGeometry():
    try:
        if not idBrickDetail():
            return

        # --- Find 'Exterior' body ---
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == 'Exterior':
                exterior_body = body
                break
        if not exterior_body:
            ui.messageBox("Error: 'Exterior' body not found.")
            return

        # --- Collect flat faces (thin in Y) ---
        flat_faces = [f for f in exterior_body.faces if abs(f.boundingBox.maxPoint.y - f.boundingBox.minPoint.y) < 0.002]
        if not flat_faces:
            ui.messageBox("Error: No flat faces found on 'Exterior'.")
            return

        # --- Find lowest Y level ---
        TOLERANCE = 0.002
        lowest_y = min(f.boundingBox.minPoint.y for f in flat_faces)
        bottom_faces = [f for f in flat_faces if abs(f.boundingBox.minPoint.y - lowest_y) < TOLERANCE]
        if not bottom_faces:
            ui.messageBox("Error: No bottom faces at lowest Y level.")
            return

        # --- Activate CAM environment ---
        ui.workspaces.itemById('CAMEnvironment').activate()
        cam = adsk.cam.CAM.cast(app.activeProduct)
        setup = cam.setups.itemByName('Charles')
        if not setup:
            ui.messageBox("Error: Setup 'Charles' not found.")
            return

        # --- Helper functions ---
        def vertex_key(v):
            p = v.geometry
            return (round(p.x, 6), round(p.y, 6), round(p.z, 6))

        def edge_key(e):
            v1 = vertex_key(e.startVertex)
            v2 = vertex_key(e.endVertex)
            return tuple(sorted([v1, v2]))

        def chain_centroid(chain_edges):
            seen = set()
            xs, zs = [], []
            for e in chain_edges:
                for v in [e.startVertex, e.endVertex]:
                    vk = vertex_key(v)
                    if vk in seen:
                        continue
                    seen.add(vk)
                    p = v.geometry
                    xs.append(p.x)
                    zs.append(p.z)
            return (sum(xs) / len(xs), sum(zs) / len(zs)) if xs else (0.0, 0.0)

        CONNECT_DIST = in_cm(0.51)

        def dist(p1, p2):
            dx, dy, dz = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
            return math.sqrt(dx*dx + dy*dy + dz*dz)

        all_chain_infos = []

        # --- For each bottom face ---
        for bottom_face in bottom_faces:
            ref_y = bottom_face.boundingBox.minPoint.y
            offset = in_cm(0.5)
            target_y = ref_y + offset

            # Find faces at target_y
            target_faces = []
            for f in exterior_body.faces:
                bb = f.boundingBox
                face_y = bb.minPoint.y
                if abs(bb.maxPoint.y - bb.minPoint.y) < 0.002 and abs(face_y - target_y) < TOLERANCE:
                    target_faces.append(f)

            if not target_faces:
                continue

            # --- Collect bottom edges ---
            bottom_edges_info = []
            for edge in bottom_face.edges:
                v1, v2 = edge.startVertex.geometry, edge.endVertex.geometry
                bottom_edges_info.append(((v1.x, v1.y, v1.z), (v2.x, v2.y, v2.z), edge))

            # --- Match target edges to bottom edges ---
            target_edges = []
            for face in target_faces:
                for edge in face.edges:
                    v1, v2 = edge.startVertex.geometry, edge.endVertex.geometry
                    if abs(v1.y - target_y) > TOLERANCE or abs(v2.y - target_y) > TOLERANCE:
                        continue
                    for b_v1_tup, b_v2_tup, b_edge in bottom_edges_info:
                        class _P: pass
                        bp1 = _P(); bp1.x, bp1.y, bp1.z = b_v1_tup
                        bp2 = _P(); bp2.x, bp2.y, bp2.z = b_v2_tup
                        d00 = math.hypot(v1.x - bp1.x, v1.z - bp1.z)
                        d01 = math.hypot(v1.x - bp2.x, v1.z - bp2.z)
                        d10 = math.hypot(v2.x - bp1.x, v2.z - bp1.z)
                        d11 = math.hypot(v2.x - bp2.x, v2.z - bp2.z)
                        if ((d00 < CONNECT_DIST and d11 < CONNECT_DIST) or
                            (d01 < CONNECT_DIST and d10 < CONNECT_DIST)):
                            target_edges.append(edge)
                            break

            if not target_edges:
                continue

            # --- Group connected edges ---
            visited = set()
            chains = []
            for e in target_edges:
                ek = edge_key(e)
                if ek in visited:
                    continue
                stack = [e]
                chain_edges = []
                while stack:
                    cur = stack.pop()
                    cur_key = edge_key(cur)
                    if cur_key in visited:
                        continue
                    visited.add(cur_key)
                    chain_edges.append(cur)
                    cur_v1, cur_v2 = cur.startVertex.geometry, cur.endVertex.geometry
                    for nxt in target_edges:
                        nk = edge_key(nxt)
                        if nk in visited:
                            continue
                        n_v1, n_v2 = nxt.startVertex.geometry, nxt.endVertex.geometry
                        if (dist(cur_v1, n_v1) < CONNECT_DIST or
                            dist(cur_v1, n_v2) < CONNECT_DIST or
                            dist(cur_v2, n_v1) < CONNECT_DIST or
                            dist(cur_v2, n_v2) < CONNECT_DIST):
                            stack.append(nxt)
                if chain_edges:
                    chains.append(chain_edges)

            # --- Sort chains around centroid for consistency ---
            all_x = [v.geometry.x for e in target_edges for v in [e.startVertex, e.endVertex]]
            all_z = [v.geometry.z for e in target_edges for v in [e.startVertex, e.endVertex]]
            xmin, xmax, zmin, zmax = min(all_x), max(all_x), min(all_z), max(all_z)
            xmid, zmid = (xmin + xmax) / 2.0, (zmin + zmax) / 2.0
            start_angle = math.atan2(zmin - zmid, xmax - xmid)
            for chain_edges in chains:
                cx, cz = chain_centroid(chain_edges)
                angle = math.atan2(cz - zmid, cx - xmid)
                delta = (start_angle - angle + 2.0 * math.pi) % (2.0 * math.pi)
                all_chain_infos.append((delta, chain_edges))

        if not all_chain_infos:
            return

        # --- Apply all chains to 'Brick Feature FM' operation ---
        op = setup.operations.itemByName('Brick Feature FM')
        if not op:
            ui.messageBox("Error: Operation 'Brick Feature FM' not found.")
            return

        contourParam = op.parameters.itemByName('contours').value
        curveSelections = contourParam.getCurveSelections()
        curveSelections.clear()

        all_chain_infos.sort(key=lambda t: t[0])

        for _, chain_edges in all_chain_infos:
            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)
            chain.inputGeometry = [e for e in chain_edges]

        contourParam.applyCurveSelections(curveSelections)
        cam.generateToolpath(op)

    except:
        ui.messageBox(f"brickFeatureFMGeometry(): failed:\n{traceback.format_exc()}")

def bumpCleanUpGeometry():
    try:
        if eastBump == True:
            # ---- CONFIG ----
            BODY_NAME = "Exterior"
            SETUP_NAME = "Charles"
            OP_NAME = "Bump Clean Up FM"
            COORD_TOL = 1e-4

            # ---- GET DESIGN ----
            ui.workspaces.itemById("FusionSolidEnvironment").activate()
            design = adsk.fusion.Design.cast(app.activeProduct)
            rootComp = design.rootComponent

            # ---- GET BODY ----
            body = None
            for b in rootComp.bRepBodies:
                if b.name == BODY_NAME:
                    body = b
                    break
            if not body:
                ui.messageBox(f"Body '{BODY_NAME}' not found.")
                return

            # ---- FIND EXTREMES ----
            xmax = max(v.geometry.x for v in body.vertices)
            ymin = min(v.geometry.y for v in body.vertices)

            # ---- FIND EDGE ----
            target_edge = None
            for face in body.faces:
                for edge in face.edges:
                    p1 = edge.startVertex.geometry
                    p2 = edge.endVertex.geometry
                    if (abs(p1.x - xmax) < COORD_TOL and abs(p2.x - xmax) < COORD_TOL and
                        abs(p1.y - ymin) < COORD_TOL and abs(p2.y - ymin) < COORD_TOL):
                        target_edge = edge
                        break
                if target_edge:
                    break

            if not target_edge:
                ui.messageBox("No edge found with both vertices on X max and Y min.")
                return

            # ---- GET CAM SETUP AND OP ----
            doc = app.activeDocument
            cam_product = doc.products.itemByProductType('CAMProductType')
            cam = adsk.cam.CAM.cast(cam_product)
            ui.workspaces.itemById("CAMEnvironment").activate()

            setup = cam.setups.itemByName(SETUP_NAME)
            if not setup:
                ui.messageBox(f"Setup '{SETUP_NAME}' not found.")
                return

            op = setup.operations.itemByName(OP_NAME)
            if not op:
                ui.messageBox(f"Operation '{OP_NAME}' not found.")
                return

            # ---- GET CHAIN PARAMETER ----
            contourParam = None
            for pname in ['curves', 'contours']:
                param = op.parameters.itemByName(pname)
                if param:
                    try:
                        contourParam = param.value
                        break
                    except:
                        pass
            if not contourParam:
                for attr in ['contourParameters', 'chainCurves']:
                    try:
                        contourParam = getattr(op, attr)
                        if contourParam:
                            break
                    except:
                        pass
            if not contourParam:
                ui.messageBox(f"Could not find chain parameter for '{OP_NAME}'.")
                return

            # ---- APPLY EDGE TO OPERATION ----
            try:
                curveSelections = contourParam.getCurveSelections()
            except:
                curveSelections = contourParam

            curveSelections.clear()
            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.reverse = False
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)
            chain.inputGeometry = [target_edge]
            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)
            
    except:
        ui.messageBox(f"bumpCleanUpGeometry(): failed:\n{traceback.format_exc()}")

def eastReturnEMGeometry():
    try:
        if east_return_result == adsk.core.DialogResults.DialogYes:
            COORD_TOL = 1e-4
            OFFSET_TOL = in_cm(0.51)  # 0.5 inch tolerance in cm

            # --- Find 'Exterior' body ---
            exterior_body = None
            for body in rootComp.bRepBodies:
                if body.name == "Exterior":
                    exterior_body = body
                    break
            if not exterior_body:
                ui.messageBox("Error: 'Exterior' body not found.")
                return

            # --- Determine max X ---
            all_x = [v.geometry.x for v in exterior_body.vertices]
            max_x = max(all_x)

            # --- Step 1: Collect edges where BOTH vertices are within 0.5in of max X ---
            edges_near_xmax = []
            for edge in exterior_body.edges:
                v1 = edge.startVertex.geometry
                v2 = edge.endVertex.geometry
                if abs(v1.x - max_x) <= OFFSET_TOL and abs(v2.x - max_x) <= OFFSET_TOL:
                    edges_near_xmax.append(edge)

            if not edges_near_xmax:
                ui.messageBox("No edges found within 0.5in of max X.")
                return

            # --- Step 2: Compute Y/Z extrema from those edges ---
            all_y = []
            all_z = []
            for e in edges_near_xmax:
                for v in [e.startVertex.geometry, e.endVertex.geometry]:
                    all_y.append(v.y)
                    all_z.append(v.z)

            y_max = max(all_y)
            z_max = max(all_z)
            z_min = min(all_z)

            # --- Step 3: Filter for edges fully on Zmax, Zmin, or Ymax ---
            selected_edges = []
            seen = set()

            def edge_key(e):
                v1, v2 = e.startVertex.geometry, e.endVertex.geometry
                return tuple(sorted([
                    (round(v1.x,5), round(v1.y,5), round(v1.z,5)),
                    (round(v2.x,5), round(v2.y,5), round(v2.z,5))
                ]))

            for e in edges_near_xmax:
                v1 = e.startVertex.geometry
                v2 = e.endVertex.geometry

                # skip if both vertices are at y-min of entire body
                body_ymin = min(v.geometry.y for v in exterior_body.vertices)
                if abs(v1.y - body_ymin) < COORD_TOL and abs(v2.y - body_ymin) < COORD_TOL:
                    continue

                if (
                    (abs(v1.z - z_max) < COORD_TOL and abs(v2.z - z_max) < COORD_TOL)
                    or (abs(v1.z - z_min) < COORD_TOL and abs(v2.z - z_min) < COORD_TOL)
                    or (abs(v1.y - y_max) < COORD_TOL and abs(v2.y - y_max) < COORD_TOL)
                ):
                    k = edge_key(e)
                    if k not in seen:
                        seen.add(k)
                        selected_edges.append(e)

            if not selected_edges:
                ui.messageBox("No edges found for Return EM (after filtering).")
                return

            # --- Step 4: Apply to CAM operation ---
            ui.workspaces.itemById("CAMEnvironment").activate()
            cam = adsk.cam.CAM.cast(app.activeProduct)
            setup = cam.setups.itemByName("Charles")
            if not setup:
                ui.messageBox("Setup 'Charles' not found.")
                return

            op = setup.operations.itemByName("Return EM")
            if not op:
                ui.messageBox("Operation 'Return EM' not found.")
                return

            contourParam = op.parameters.itemByName("contours").value
            curveSelections = contourParam.getCurveSelections()
            curveSelections.clear()

            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.reverse = False
            chain.startExtensionLength = in_mm(3.0)
            chain.endExtensionLength = in_mm(3.0)
            chain.inputGeometry = selected_edges

            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)

    except:
        ui.messageBox(f"eastReturnGeometry(): failed:\n{traceback.format_exc()}")

def eastReturnGeometry():
    try:
        if east_return_result == adsk.core.DialogResults.DialogYes:
            # --- CONFIG ---
            OFFSET_IN = 0.5
            COORD_TOL = 1e-4

            offset = in_cm(OFFSET_IN)

            # --- FIND BODY ---
            exterior_body = None
            for body in rootComp.bRepBodies:
                if body.name == 'Exterior':
                    exterior_body = body
                    break

            # --- FIND PLANAR FACES PARALLEL TO YZ (normal ±X) ---
            max_x = max(v.geometry.x for v in exterior_body.vertices)
            target_faces = []

            for face in exterior_body.faces:
                geom = face.geometry
                if not isinstance(geom, adsk.core.Plane):
                    continue

                evaluator = face.evaluator
                param = adsk.core.Point2D.create(0.5, 0.5)
                success, normal_vec = evaluator.getNormalAtParameter(param)
                if not success:
                    continue

                # Ensure normal is mostly ±X (parallel to YZ)
                if abs(normal_vec.x) < 0.999:
                    continue

                face_x = face.boundingBox.maxPoint.x
                if abs(face_x - max_x) < COORD_TOL or abs(face_x - (max_x - offset)) < COORD_TOL:
                    target_faces.append(face)

            if not target_faces:
                ui.messageBox("No faces found parallel to YZ at max X or 0.5in behind.")
                return

            # --- SWITCH TO CAM WORKSPACE ---
            ui.workspaces.itemById("CAMEnvironment").activate()

            # --- GET CAM SETUP AND OP ---
            doc = app.activeDocument
            cam_product = doc.products.itemByProductType('CAMProductType')
            cam = adsk.cam.CAM.cast(cam_product)
            setup = cam.setups.itemByName('Charles')
            op = setup.operations.itemByName('Return FM')

            # --- GET FACE PARAMETER ---
            contourParam: adsk.cam.CadContours2dParameterValue = op.parameters.itemByName('stockContours').value
            curveSelections = contourParam.getCurveSelections()
            curveSelections.clear()

            # --- CREATE FACE SELECTIONS ---
            new_selection_list = adsk.core.ObjectCollection.create()
            for face in target_faces:
                fc: adsk.cam.FaceContourSelection = curveSelections.createNewFaceContourSelection()
                fc.loopType = adsk.cam.LoopTypes.AllLoops
                fc.sideType = adsk.cam.SideTypes.StartOutsideSideType
                fc.inputGeometry = [face]
                new_selection_list.add(fc)

            curveSelections.curveSelections = new_selection_list
            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)
         
    except:
        ui.messageBox(f"eastReturnGeometry(): failed:\n{traceback.format_exc()}")

def eastReturnBrickGeometry():
    try:
        # --- Run only if one of them is true ---
        if not (idEastReturnBrickDetail() or idWestReturnBrickDetail()):
            #ui.messageBox("No return brick detail active.")
            return

        # --- Switch to Fusion workspace ---
        ui.workspaces.itemById("FusionSolidEnvironment").activate()
        design = adsk.fusion.Design.cast(app.activeProduct)
        rootComp = design.rootComponent
        COORD_TOL = 1e-4

        # --- FIND BODY ---
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":
                exterior_body = body
                break
        if not exterior_body:
            ui.messageBox("No 'Exterior' body found.")
            return

        # --- Determine whether we’re doing East or West ---
        find_max_x = idEastReturnBrickDetail
        find_min_x = idWestReturnBrickDetail

        all_x = [v.geometry.x for v in exterior_body.vertices]
        target_x = max(all_x) if find_max_x else min(all_x)

        # --- FIND TARGET FACE (parallel to YZ plane, at target X) ---
        target_face = None
        for face in exterior_body.faces:
            geom = face.geometry
            if not isinstance(geom, adsk.core.Plane):
                continue

            evaluator = face.evaluator
            param = adsk.core.Point2D.create(0.5, 0.5)
            success, normal_vec = evaluator.getNormalAtParameter(param)
            if not success:
                continue

            # We only want faces roughly parallel to YZ (normal ±X)
            if abs(normal_vec.x) < 0.999:
                continue

            face_x = (
                face.boundingBox.maxPoint.x if find_max_x else face.boundingBox.minPoint.x
            )
            if abs(face_x - target_x) < COORD_TOL:
                target_face = face
                break

        if not target_face:
            ui.messageBox(
                f"No face found at {'max' if find_max_x else 'min'} X ({target_x:.3f})."
            )
            return

        # --- FIND Y-ORIENTED EDGES ON THAT FACE ---
        y_edges = []
        for edge in target_face.edges:
            sv = edge.startVertex.geometry
            ev = edge.endVertex.geometry
            dx = abs(ev.x - sv.x)
            dy = abs(ev.y - sv.y)
            dz = abs(ev.z - sv.z)
            if dy > dx and dy > dz:
                y_edges.append(edge)

        if len(y_edges) < 2:
            ui.messageBox(
                f"Found only {len(y_edges)} Y-oriented edges on target face at X={target_x:.3f}."
            )
            return

        # --- SWITCH TO CAM WORKSPACE ---
        ui.workspaces.itemById("CAMEnvironment").activate()

        # --- GET SETUP AND OP ---
        doc = app.activeDocument
        cam_product = doc.products.itemByProductType("CAMProductType")
        cam = adsk.cam.CAM.cast(cam_product)
        setup = cam.setups.itemByName("Charles")
        if not setup:
            ui.messageBox("Setup 'Charles' not found.")
            return

        op = setup.operations.itemByName("Return Brick FM")
        if not op:
            ui.messageBox("Operation 'Return Brick FM' not found.")
            return

        # --- APPLY CHAINS ---
        contourParam: adsk.cam.CadContours2dParameterValue = op.parameters.itemByName(
            "contours"
        ).value

        try:
            curveSelections = contourParam.getCurveSelections()
        except:
            curveSelections = contourParam

        curveSelections.clear()

        for edge in y_edges[:2]:  # Only use two Y-axis edges
            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.reverse = False
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)
            chain.inputGeometry = [edge]

        contourParam.applyCurveSelections(curveSelections)
        cam.generateToolpath(op)

    except:
        ui.messageBox(f"eastReturnBrickGeometry(): failed:\n{traceback.format_exc()}")

def westReturnEMGeometry():
    try:
        if west_return_result == adsk.core.DialogResults.DialogYes:
            COORD_TOL = 1e-4
            OFFSET_TOL = in_cm(0.51)  # 0.5 inch tolerance in cm

            # --- Find 'Exterior' body ---
            exterior_body = None
            for body in rootComp.bRepBodies:
                if body.name == "Exterior":
                    exterior_body = body
                    break
            if not exterior_body:
                ui.messageBox("Error: 'Exterior' body not found.")
                return

            # --- Determine min X ---
            all_x = [v.geometry.x for v in exterior_body.vertices]
            min_x = min(all_x)

            # --- Step 1: Collect edges where BOTH vertices are within 0.5in of min X ---
            edges_near_xmin = []
            for edge in exterior_body.edges:
                v1 = edge.startVertex.geometry
                v2 = edge.endVertex.geometry
                if abs(v1.x - min_x) <= OFFSET_TOL and abs(v2.x - min_x) <= OFFSET_TOL:
                    edges_near_xmin.append(edge)

            if not edges_near_xmin:
                ui.messageBox("No edges found within 0.5in of min X.")
                return

            # --- Step 2: Compute Y/Z extrema from those edges ---
            all_y = []
            all_z = []
            for e in edges_near_xmin:
                for v in [e.startVertex.geometry, e.endVertex.geometry]:
                    all_y.append(v.y)
                    all_z.append(v.z)

            y_max = max(all_y)
            z_max = max(all_z)
            z_min = min(all_z)

            # --- Step 3: Filter for edges fully on Zmax, Zmin, or Ymax ---
            selected_edges = []
            seen = set()

            def edge_key(e):
                v1, v2 = e.startVertex.geometry, e.endVertex.geometry
                return tuple(sorted([
                    (round(v1.x,5), round(v1.y,5), round(v1.z,5)),
                    (round(v2.x,5), round(v2.y,5), round(v2.z,5))
                ]))

            for e in edges_near_xmin:
                v1 = e.startVertex.geometry
                v2 = e.endVertex.geometry

                # skip if both vertices are at y-min of entire body
                body_ymin = min(v.geometry.y for v in exterior_body.vertices)
                if abs(v1.y - body_ymin) < COORD_TOL and abs(v2.y - body_ymin) < COORD_TOL:
                    continue

                if (
                    (abs(v1.z - z_max) < COORD_TOL and abs(v2.z - z_max) < COORD_TOL)
                    or (abs(v1.z - z_min) < COORD_TOL and abs(v2.z - z_min) < COORD_TOL)
                    or (abs(v1.y - y_max) < COORD_TOL and abs(v2.y - y_max) < COORD_TOL)
                ):
                    k = edge_key(e)
                    if k not in seen:
                        seen.add(k)
                        selected_edges.append(e)

            if not selected_edges:
                ui.messageBox("No edges found for Return EM (after filtering).")
                return

            # --- Step 4: Apply to CAM operation ---
            ui.workspaces.itemById("CAMEnvironment").activate()
            cam = adsk.cam.CAM.cast(app.activeProduct)
            setup = cam.setups.itemByName("Charles")
            if not setup:
                ui.messageBox("Setup 'Charles' not found.")
                return

            op = setup.operations.itemByName("Return EM")
            if not op:
                ui.messageBox("Operation 'Return EM' not found.")
                return

            contourParam = op.parameters.itemByName("contours").value
            curveSelections = contourParam.getCurveSelections()
            curveSelections.clear()

            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.isReverted = True
            chain.startExtensionLength = in_mm(3.0)
            chain.endExtensionLength = in_mm(3.0)
            chain.inputGeometry = selected_edges

            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)

    except:
        ui.messageBox(f"westReturnEMGeometry(): failed:\n{traceback.format_exc()}")

def westReturnGeometry():
    try:
        if west_return_result == adsk.core.DialogResults.DialogYes:
            # --- CONFIG ---
            OFFSET_IN = 0.5
            COORD_TOL = 1e-4

            offset = in_cm(OFFSET_IN)

            # --- FIND BODY ---
            exterior_body = None
            for body in rootComp.bRepBodies:
                if body.name == 'Exterior':
                    exterior_body = body
                    break

            # --- FIND PLANAR FACES PARALLEL TO YZ (normal ±X) ---
            min_x = min(v.geometry.x for v in exterior_body.vertices)
            target_faces = []

            for face in exterior_body.faces:
                geom = face.geometry
                if not isinstance(geom, adsk.core.Plane):
                    continue

                evaluator = face.evaluator
                param = adsk.core.Point2D.create(0.5, 0.5)
                success, normal_vec = evaluator.getNormalAtParameter(param)
                if not success:
                    continue

                # Ensure normal is mostly ±X (parallel to YZ)
                if abs(normal_vec.x) < 0.999:
                    continue

                face_x = face.boundingBox.minPoint.x
                if abs(face_x - min_x) < COORD_TOL or abs(face_x - (min_x + offset)) < COORD_TOL:
                    target_faces.append(face)

            if not target_faces:
                ui.messageBox("No faces found parallel to YZ at max X or 0.5in behind.")
                return

            # --- SWITCH TO CAM WORKSPACE ---
            ui.workspaces.itemById("CAMEnvironment").activate()

            # --- GET CAM SETUP AND OP ---
            doc = app.activeDocument
            cam_product = doc.products.itemByProductType('CAMProductType')
            cam = adsk.cam.CAM.cast(cam_product)
            setup = cam.setups.itemByName('Charles')
            op = setup.operations.itemByName('Return FM')

            # --- GET FACE PARAMETER ---
            contourParam: adsk.cam.CadContours2dParameterValue = op.parameters.itemByName('stockContours').value
            curveSelections = contourParam.getCurveSelections()
            curveSelections.clear()

            # --- CREATE FACE SELECTIONS ---
            new_selection_list = adsk.core.ObjectCollection.create()
            for face in target_faces:
                fc: adsk.cam.FaceContourSelection = curveSelections.createNewFaceContourSelection()
                fc.loopType = adsk.cam.LoopTypes.AllLoops
                fc.sideType = adsk.cam.SideTypes.StartOutsideSideType
                fc.inputGeometry = [face]
                new_selection_list.add(fc)

            curveSelections.curveSelections = new_selection_list
            contourParam.applyCurveSelections(curveSelections)
            cam.generateToolpath(op)
         
    except:
        ui.messageBox(f"westReturnGeometry(): failed:\n{traceback.format_exc()}")

def westReturnBrickGeometry():
    try:
        # --- Run only if one of them is true ---
        if not idWestReturnBrickDetail():
            #ui.messageBox("No return brick detail active.")
            return

        # --- Switch to Fusion workspace ---
        ui.workspaces.itemById("FusionSolidEnvironment").activate()
        design = adsk.fusion.Design.cast(app.activeProduct)
        rootComp = design.rootComponent
        COORD_TOL = 1e-4

        # --- FIND BODY ---
        exterior_body = None
        for body in rootComp.bRepBodies:
            if body.name == "Exterior":
                exterior_body = body
                break
        if not exterior_body:
            ui.messageBox("No 'Exterior' body found.")
            return

        # --- Determine whether we’re doing East or West ---
        find_min_x = idWestReturnBrickDetail

        all_x = [v.geometry.x for v in exterior_body.vertices]
        target_x = min(all_x) if find_min_x else min(all_x)

        # --- FIND TARGET FACE (parallel to YZ plane, at target X) ---
        target_face = None
        for face in exterior_body.faces:
            geom = face.geometry
            if not isinstance(geom, adsk.core.Plane):
                continue

            evaluator = face.evaluator
            param = adsk.core.Point2D.create(0.5, 0.5)
            success, normal_vec = evaluator.getNormalAtParameter(param)
            if not success:
                continue

            # We only want faces roughly parallel to YZ (normal ±X)
            if abs(normal_vec.x) < 0.999:
                continue

            face_x = (
                face.boundingBox.minPoint.x if find_min_x else face.boundingBox.minPoint.x
            )
            if abs(face_x - target_x) < COORD_TOL:
                target_face = face
                break

        if not target_face:
            ui.messageBox(
                f"No face found at {'min' if find_min_x else 'min'} X ({target_x:.3f})."
            )
            return

        # --- FIND Y-ORIENTED EDGES ON THAT FACE ---
        y_edges = []
        for edge in target_face.edges:
            sv = edge.startVertex.geometry
            ev = edge.endVertex.geometry
            dx = abs(ev.x - sv.x)
            dy = abs(ev.y - sv.y)
            dz = abs(ev.z - sv.z)
            if dy > dx and dy > dz:
                y_edges.append(edge)

        if len(y_edges) < 2:
            ui.messageBox(
                f"Found only {len(y_edges)} Y-oriented edges on target face at X={target_x:.3f}."
            )
            return

        # --- SWITCH TO CAM WORKSPACE ---
        ui.workspaces.itemById("CAMEnvironment").activate()

        # --- GET SETUP AND OP ---
        doc = app.activeDocument
        cam_product = doc.products.itemByProductType("CAMProductType")
        cam = adsk.cam.CAM.cast(cam_product)
        setup = cam.setups.itemByName("Charles")
        if not setup:
            ui.messageBox("Setup 'Charles' not found.")
            return

        op = setup.operations.itemByName("Return Brick FM")
        if not op:
            ui.messageBox("Operation 'Return Brick FM' not found.")
            return

        # --- APPLY CHAINS ---
        contourParam: adsk.cam.CadContours2dParameterValue = op.parameters.itemByName(
            "contours"
        ).value

        try:
            curveSelections = contourParam.getCurveSelections()
        except:
            curveSelections = contourParam

        curveSelections.clear()

        for edge in y_edges[:2]:  # Only use two Y-axis edges
            chain = curveSelections.createNewChainSelection()
            chain.isOpen = True
            chain.reverse = False
            chain.startExtensionLength = in_mm(5.0)
            chain.endExtensionLength = in_mm(5.0)
            chain.inputGeometry = [edge]

        contourParam.applyCurveSelections(curveSelections)
        cam.generateToolpath(op)

    except:
        ui.messageBox(f"westReturnBrickGeometry(): failed:\n{traceback.format_exc()}")

def showBodies():
    try:
        bodies = rootComp.bRepBodies
    
        prefixes_to_show = ('Bump') # You can add more names, Example: ('Stud', 'Track', 'Bump')
        for body in bodies: 
            # Check if the body name starts with any of the required prefixes
            if body.name.startswith(prefixes_to_show):
                body.isVisible = True
             
            if body.name == "Sheathing":
                body.isVisible = True

            if body.name == "Exterior":
                body.isVisible = True
    
    except:
        ui.messageBox(f"showBodies(): failed:\n{traceback.format_exc()}")

def errorDetection(): 
    try: 
        # Find the edge of the stud body for our WCS
        sheathing = [body for body in rootComp.bRepBodies if body.name.startswith("Sheathing")]

        if any("Sheathing" in body.name for body in rootComp.bRepBodies):

            # Initialize min and max values
            min_x, min_y, min_z = float('inf'), float('inf'), float('inf')
            max_x, max_y, max_z = float('-inf'), float('-inf'), float('-inf')

            # Loop through all "Sheathing" bodies and expand bounds
            for body in sheathing:
                box = body.boundingBox
                min_x = min(min_x, box.minPoint.x)
                min_y = min(min_y, box.minPoint.y)
                min_z = min(min_z, box.minPoint.z)
                max_x = max(max_x, box.maxPoint.x)
                max_y = max(max_y, box.maxPoint.y)
                max_z = max(max_z, box.maxPoint.z)

            # Create max and min "Sheathing" points.
            sheathing_max_point = adsk.core.Point3D.create(max_x, max_y, max_z)
            sheathing_min_point = adsk.core.Point3D.create(min_x, min_y, min_z)
            
            # Compare "Sheathing" and "Track" max and min values.
            max_z_result = abs(abs(sheathing_max_point.z) - abs(track_max_point.z))
            if max_z_result > .002:
                max_z_result = (max_z_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Frame' detected. \n \u2022 Z max: {max_z_result:.3f}in")

            min_z_result = abs(abs(sheathing_min_point.z) - abs(track_min_point.z))
            if min_z_result > .002: 
                min_z_result = (min_z_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Frame' detected. \n \u2022 Z min: {min_z_result:.3f}in")

            max_x_result = abs(abs(sheathing_max_point.x) - abs(track_max_point.x))
            if max_x_result > .002: 
                max_x_result = (max_x_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Frame' detected. \n \u2022 X max: {max_x_result:.3f}in")

            min_x_result = abs(abs(sheathing_min_point.x) - abs(track_min_point.x))
            if min_x_result > .002: 
                min_x_result = (min_x_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Frame' detected. \n \u2022 X min: {min_x_result:.3f}in")

        if any("Foam" in body.name for body in rootComp.bRepBodies):
            # foam = [body for body in rootComp.bRepBodies if body.name.startswith("Foam")]
            for body in rootComp.bRepBodies:
                if body.name == "Foam":
                    foamBody = body

            # Get the bounding boxes for each body
            foam = foamBody.boundingBox

            # Compare "Sheathing" and "Foam" max and min values.
            foam_min_z_result = abs(abs(sheathing_min_point.z) - abs(foam.minPoint.z))
            if foam_min_z_result > .002: 
                foam_min_z_result = (foam_min_z_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Foam' detected. \n \u2022 Z min: {foam_min_z_result:.3f}in")

            foam_max_z_result = abs(abs(sheathing_max_point.z) - abs(foam.maxPoint.z))
            if foam_max_z_result > .002:
                foam_max_z_result = (foam_max_z_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Foam' detected. \n \u2022 Z max: {foam_max_z_result:.3f}in")

            foam_min_x_result = (abs(abs(sheathing_min_point.x)) - (abs(foam.minPoint.x)))
            if foam_min_x_result > .002: 
                foam_min_x_result = (foam_min_x_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Foam' detected. \n \u2022 X min: {foam_min_x_result:.3f}in")
                
            foam_max_x_result = abs(abs(sheathing_max_point.x) - abs(foam.maxPoint.x))
            if foam_max_x_result > .002: 
                foam_max_x_result = (foam_max_x_result / 2.54)
                addMessage(f"Difference between 'Sheathing' and 'Foam' detected. \n \u2022 X max: {foam_max_x_result:.3f}in")
            
    except:
        ui.messageBox(f"ErrorDetection(): failed:\n{traceback.format_exc()}")

def scriptSummary():
    try:
        if len(script_summary) == 0:
            addMessage("Everything looks great!")

        full_message = "\n".join(script_summary)
        ui.messageBox(full_message, "Script Summary", 
                    adsk.core.MessageBoxButtonTypes.OKButtonType,
                    adsk.core.MessageBoxIconTypes.InformationIconType)
    except:
        ui.messageBox(f"scriptSummary(): failed:\n{traceback.format_exc()}")
