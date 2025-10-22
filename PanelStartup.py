# Script version 1.0 

# TODO: Create script to automate cutting the "L" shape notches in the back of the sheathing.
# TODO: Need to understand when bumps are framed at Melvin vs after. Then change bumpEast() and bumpWest() cut the new copy of sheathing.
# TODO: May need to add an easily to change variable for return diminesion in case it changes from project to project.
# TODO: Add logic for identifying bumps and catch all. See 8&L Panel 10014.
# TODO: Change name of idFrame
# TODO: errorDetection() function is not working correctly.

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
    rotateBodiesToFront()   # Rotates the bodies around the Z axis so the front of the panel is the front view.
    moveBodiesToOrgin()     # Moves all bodies to the origin.
    openPDF()               # Opens the PDF drawing for the panel.
    stockBody()             # Create a stock body for Charles setup.
    changeUnits()           # Change units to inches.
    idFoam()                # Identify and rename foam bodies.
    idFrame()               # Identify and rename frame bodies.
    idStuds()               # Identify and rename stud bodies.
    idTrack()               # Identify and rename track bodies.
    idRotatedBump()         # Identify and rename bump bodies that are on an angle.
    idBump()                # Identify and rename bump bodies.
    idWindows()             # Identifies windows and determines if the window bevel toolpath is needed.
    idOrigin()              # Identifies WCS X and Y axis from the stud and track bodies.
    idReturn()              # Is there a return on the right side of the panel that would interfer with WCS?
    melvinOrgin()           # Creates a sketch and contruction point for the Melvin WCS.
    charlesOrgin()          # Creates a sketch and contruction point for the Charles WCS.
    mergeSheathing()        # Merge all sheathing panels into one.
    camWorkspace()          # Create the cam workspace.
    melvinSetup()           # Create the Melvin setup.
    charlesSetup()          # Create the Charles setup.
    idThinFoam()            # Checks for thin foam and adjusts the adjust toolpath as necessary.
    errorDetection()        # Compares 'Frame' and 'Sheathing' and 'Foam' and 'Sheathing' to detect errors from Revit export.
    scriptSummary()         # Displays a summary at the end of the script.

def addMessage(msg):        # This function adds messages throughout the script to give a summary at the end.
    try:
        script_summary.append(f"\u2022 {msg}\n")
    except:
        ui.messageBox(f"addMessage(): failed:\n{traceback.format_exc()}")

def in_cm(x):               # This function converts inches to cm.
    return x * 2.54

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

def moveBodiesToOrgin():
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
        ui.messageBox(f"moveBodiesToOrgin(): failed:\n{traceback.format_exc()}")
    
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
        global foamresult
        foamresult = False
        
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
            foamresult = True
    except:
        ui.messageBox(f"identifyFoam(): failed:\n{traceback.format_exc()}")

def idFrame():
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
                body.name = "Frame"
    except:
        ui.messageBox(f"identifyFrame(): failed:\n{traceback.format_exc()}")

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

def idRotatedBump():        # This function works but is sloppy and needs to be cleaned up.
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
        ui.messageBox(f"identifyBump(): failed:\n{traceback.format_exc()}")

def idWindows():            # This function tries to detect if the panel has windows. Goal is to determine to add the window bevel toolpath or not.
    
    # Will need to change logic so that it does not pick up small holes or air vents.
    global windows
    windows = False
    loopTest = False

    try:
        target_y = -6.625     # inches
        tolerance = 0.01      # inches tolerance

        unitsMgr = design.unitsManager
        target_y_cm = unitsMgr.evaluateExpression(f"{target_y} in", "cm")
        tol_cm = unitsMgr.evaluateExpression(f"{tolerance} in", "cm")

        for body in rootComp.bRepBodies:
            if body.name != "Exterior":
                continue  # only check the sheathing body

            #ui.messageBox(f'Checking body: {body.name}')

            for face in body.faces:
                geom = face.geometry
                if not isinstance(geom, adsk.core.Plane):
                    continue  # skip non-planar faces

                normal = geom.normal
                origin = geom.origin

                # ZX plane → normal should be along ±Y
                if abs(abs(normal.y) - 1.0) > 1e-3:
                    continue

                # Check for the plane located at Y = -6.625 in (within tolerance)
                if abs(origin.y - target_y_cm) > tol_cm:
                    continue

                #ui.messageBox(f'Found ZX face at Y={origin.y:.4f} cm (~{target_y} in)')

                # Detect holes/voids on this face
                loops = face.loops
                for loop_index in range(loops.count - 1):
                    loop = loops.item(loop_index)

                    if loop:
                        loopTest = True
              
                    min_x_cm = float('inf')
                    max_x_cm = float('-inf')
                    for edge in loop.edges:
                        edge_bBox = edge.boundingBox
                        
                        # Update the overall min/max X for the entire loop
                        min_x_cm = min(min_x_cm, edge_bBox.minPoint.x)
                        max_x_cm = max(max_x_cm, edge_bBox.maxPoint.x)
                    
                    # Store the extents (using the first window found for now)
                    window_x_extents = (min_x_cm, max_x_cm)

                    # --- Output for Debugging (similar to your original code) ---
                    edge_lengths_in = [
                        round(unitsMgr.convert(edge.length, "cm", "in"), 3) for edge in loop.edges
                    ]

                    # This is your new window location data
                    window_start_in = round(unitsMgr.convert(min_x_cm, "cm", "in"), 3)
                    window_end_in = round(unitsMgr.convert(max_x_cm, "cm", "in"), 3)
        
        if loopTest == True:
            for body in rootComp.bRepBodies:
                if body.name == "Exterior":  
                    
                    # Get bounding box for X limits
                    bb = body.boundingBox
                    minX = bb.minPoint.x + window_start_in
                    maxX = bb.maxPoint.x - window_end_in

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
                        windows = True
                        #addMessage("A window with foam under it was detected.")
        
        if windows == False:
            #addMessage("A window without foam under it was detected.")
            None
        # return windows

    except:
        ui.messageBox('identifyWindows() Failed:\n{}'.format(traceback.format_exc()))

def bumpEast():             # This function cuts the "Stock" and "Foam" bodies to prevent the facinghead from interfeering with the bump.
    try:
        # Hide all bodies in the root component
        for body in rootComp.bRepBodies:
            body.isVisible = False

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
        if foamresult == True:
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

def bumpWest():             # This function cuts the "Stock" and "Foam" bodies to prevent the facinghead from interfeering with the bump.
    try:
        # Hide all bodies in the root component
        for body in rootComp.bRepBodies:
            body.isVisible = False

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
        if foamresult == True:
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
    except:
        ui.messageBox(f"{traceback.format_exc()}", "openPDF(): failed!")

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
 
    # The script can only idenify a return if there are "Stud" bodies in the model.
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

def melvinOrgin():
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

        # This tries to set the WCS X axis to location ideneified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if east_return_result == adsk.core.DialogResults.DialogYes:
            if any("Stud" in body.name for body in rootComp.bRepBodies):
                offset_x = stud_max_point.x
                difference = abs((abs(max_x)) - (abs(stud_max_point.x)) / 2.54)
                if not abs(difference - 0.063) >= .002 or abs(difference - 4.625 >= .002): #Probably need sheathing width as well.
                    addMessage(f"There may be an issue with the Melvin WCS X axis. \n    \u2022 Difference between 'Stud' and 'Exterior' bodies: {difference:.3f}in")
            else:
                offset_x = corner_x - in_cm(4.6875)

        # This tries to set the WCS Y axis to location ideneified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
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

        # This tries to set the WCS X axis to location ideneified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
        if west_return_result == adsk.core.DialogResults.DialogYes:
            if any("Stud" in body.name for body in rootComp.bRepBodies):
                bl_offset_x = stud_min_point.x
                if (abs(min_x)) - (abs(stud_min_point.x)) != in_cm(0.0625) or in_cm (4.6875): #Probably need sheathing width as well.
                    addMessage("There may be an issue with the Melvin 'Flipped' WCS X axis")
            else:
                bl_offset_x = bl_corner_x + in_cm(4.6875)

        # This tries to set the WCS Y axis to location ideneified in 'identifyOrigin()' function. If not, it uses a default hardcode offset.
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
        ui.messageBox(f"melvinOrgin(): failed:\n{traceback.format_exc()}")

def charlesOrgin():
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
        ui.messageBox(f"charlesOrgin(): failed:\n{traceback.format_exc()}")

def mergeSheathingOld():
    try:
        sheathing_thickness = in_cm(0.625)
        tolerance = in_cm(0.001)

        # Keep merging until only one "Sheathing" body remains
        for _ in range(2):  # Run up to twice
            bodies_to_merge = []

            for body in rootComp.bRepBodies:
                boundingBox = body.boundingBox
                width = boundingBox.maxPoint.y - boundingBox.minPoint.y
                if abs(width - sheathing_thickness) < tolerance:
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
        ui.messageBox(f"{traceback.format_exc()}", "mergeSheathin(): failed!\n")

def mergeSheathing():
    try:
        sheathing_thickness = in_cm(0.625)
        tolerance = in_cm(0.001)
        combineFeatures = rootComp.features.combineFeatures
        all_bodies = list(rootComp.bRepBodies)

        # --- Step 1: Rename all bodies as "Sheathing" ---
        for body in rootComp.bRepBodies:
            boundingBox = body.boundingBox
            width = boundingBox.maxPoint.y - boundingBox.minPoint.y
            if abs(width - sheathing_thickness) < tolerance:
                body.name = "SheathingPanel"
                body.isVisible = False

        # --- Step 2: Make copies and rename them "SheathingCopy" ---
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
        ui.messageBox(f"{traceback.format_exc()}", "mergeSheathing(): failed!")

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
        # Excluding 10 inches from either side to avoid false positivies from 'bumps'.
        foamresult = False
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
                    foamresult = True

        if foamresult == True:
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

            if west_return_result == adsk.core.DialogResults.DialogYes:
                changeWestReturnEM = setup.operations.itemByName('Return EM')
                changeWestReturnEM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]
                changeWestReturnEM.parameters.itemByName("view_orientation_flipZ").value.value = True
                            
            if idEastReturnBrickDetail():
                changeEastReturnFM = setup.operations.itemByName('Return FM')
                changeEastReturnFM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]

                eastReturnBrick = setup.operations.itemByName('Return Brick FM')
                eastReturnBrick.parameters.itemByName('view_orientation_axisZ').value.value = [rootComp.yZConstructionPlane]

            if idWestReturnBrickDetail():
                changeWestReturnFM = setup.operations.itemByName('Return FM')
                changeWestReturnFM.parameters.itemByName("view_orientation_axisZ").value.value = [rootComp.yZConstructionPlane]
                changeWestReturnFM.parameters.itemByName("view_orientation_flipZ").value.value = True

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

def bumpMod():              # This function modifies the Facinghead toolpath if a bump is decected
    try:
        if 'Charles' in [setup.name for setup in setups]:
            for body in rootComp.bRepBodies:
                # Check if the body's name is "sheathing" (case-sensitive)
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
                addMessage(f"Difference between 'Sheating' and 'Frame' detected. \n \u2022 Z max: {max_z_result:.3f}in")

            min_z_result = abs(abs(sheathing_min_point.z) - abs(track_min_point.z))
            if min_z_result > .002: 
                min_z_result = (min_z_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Frame' detected. \n \u2022 Z min: {min_z_result:.3f}in")

            max_x_result = abs(abs(sheathing_max_point.x) - abs(track_max_point.x))
            if max_x_result > .002: 
                max_x_result = (max_x_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Frame' detected. \n \u2022 X max: {max_x_result:.3f}in")

            min_x_result = abs(abs(sheathing_min_point.x) - abs(track_min_point.x))
            if min_x_result > .002: 
                min_x_result = (min_x_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Frame' detected. \n \u2022 X min: {min_x_result:.3f}in")

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
                addMessage(f"Difference between 'Sheating' and 'Foam' detected. \n \u2022 Z min: {foam_min_z_result:.3f}in")

            foam_max_z_result = abs(abs(sheathing_max_point.z) - abs(foam.maxPoint.z))
            if foam_max_z_result > .002:
                foam_max_z_result = (foam_max_z_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Foam' detected. \n \u2022 Z max: {foam_max_z_result:.3f}in")

            foam_min_x_result = (abs(abs(sheathing_min_point.x)) - (abs(foam.minPoint.x)))
            if foam_min_x_result > .002: 
                foam_min_x_result = (foam_min_x_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Foam' detected. \n \u2022 X min: {foam_min_x_result:.3f}in")
                
            foam_max_x_result = abs(abs(sheathing_max_point.x) - abs(foam.maxPoint.x))
            if foam_max_x_result > .002: 
                foam_max_x_result = (foam_max_x_result / 2.54)
                addMessage(f"Difference between 'Sheating' and 'Foam' detected. \n \u2022 X max: {foam_max_x_result:.3f}in")
            
    except:
        ui.messageBox(f"ErrorDection(): failed:\n{traceback.format_exc()}")

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
 