# Autodesk Fusion CAM Automation Script
**Version:** 1.0  
**Author:** Andy Hughes 

---

## Overview

This Python script for Autodesk Fusion automates the essential steps for preparing panel models for CNC machining on two custom linuxCNC machines, **"Melvin"** and **"Charles."**  
It handles model cleanup, orientation, body identification, work coordinate system (WCS) placement, and the creation of initial CAM setups.

---

## Key Features

* **Model Standardization:** Rotates the model based on user selection to align the desired **Front face** with the viewport and moves all bodies to the **global origin (0, 0, 0)**.  
* **Unit Enforcement:** Changes the active design units to **Inches**.  
* **Component Identification:** Automatically identifies and renames core panel components based on thickness:

  | Component | Thickness (inches) |
  | :--- | :--- |
  | Exterior | Initial body |
  | Foam | 3.0 |
  | Stud | 6.0 |
  | Track | 6.143 |
  | Sheathing | 0.625 (copied and merged into one body) |

* **Stock Creation:** Creates and offsets a copy of the `Exterior` body named **Stock** for the Charles setup.  
* **"Bump" Handling:** Detects large structural "bumps" and performs **geometric cuts** on the `Stock` and `Foam` bodies to prevent collisions with the facing toolpath.  
* **WCS Placement:** Creates machine-specific origin points (`Point1` for **Melvin**, `Point2` for **Charles**), automatically adjusting the X-offset if a panel **"return"** is detected or manually confirmed.  
* **CAM Setup:** Switches to the **Manufacture Workspace** and creates initial CAM setups for both **Melvin** and **Charles**.  
* **Process Checks:** Checks for **thin foam** and adjusts corresponding toolpath depths; runs a **foam error detection** against sheathing dimensions.  
* **External BIM Link:** Launches a local Python BIM tool or web page based on the panel's file name.  
* **Status Reporting:** Gathers all warnings and status updates into a final message box for the user.

---

## Usage

1. **Open the Panel Model:** Ensure your raw imported model is open and active in the **Design Workspace**.  
2. **Run the Script:** Access **Scripts and Add-Ins** in Fusion 360 and run the script.  
3. **Select Front Face:** The script prompts you to select the face that should become the **Front View** (facing the camera along the negative Y-axis).  
4. **Answer Prompts:** If the script detects a potential **right-side return**, confirm or deny as appropriate for correct WCS placement.  
5. **Review Report:** A final message box summarizes any detected features, errors, or adjustments.  
6. **Select Toolpath Geometry:** The script concludes in the **Manufacture Workspace**, with the basic setups created and ready for toolpath generation.

---

## Notes and Customization

### Environment-Specific Paths
The `openBIM()` function contains **hardcoded paths** that must be updated for your environment. If paths are incorrect, the script falls back to opening a generic BIM web page.

### Assumptions (Hardcoded Dimensions)
Component detection relies on strict dimensional matching. Values must match your panel system:

| Component | Target Thickness (inches) |
| :--- | :--- |
| Foam | 3.000 |
| Stud | 6.000 |
| Track | 6.143 |
| Sheathing | 0.625 |

---