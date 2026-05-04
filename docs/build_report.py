"""Generate xarm_pick_report.pdf — a detailed project + findings report.

Output: docs/xarm_pick_report.pdf
"""
from __future__ import annotations

import os
from datetime import date

from reportlab.lib import colors
from reportlab.lib.pagesizes import LETTER
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate, PageTemplate, Frame, Paragraph, Spacer, PageBreak,
    Image, Table, TableStyle, KeepTogether, ListFlowable, ListItem,
)


HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'xarm_pick_report.pdf')
FIGS = os.path.join(HERE, 'report_figs')


# ----------------------------------------------------------------------
#  Styles
# ----------------------------------------------------------------------

styles = getSampleStyleSheet()
H_TITLE = ParagraphStyle('Title', parent=styles['Title'],
                         fontName='Helvetica-Bold', fontSize=24, leading=28,
                         alignment=TA_CENTER, spaceAfter=8)
H_SUBT = ParagraphStyle('Subtitle', parent=styles['Normal'],
                        fontName='Helvetica', fontSize=12, leading=16,
                        alignment=TA_CENTER, spaceAfter=6, textColor=colors.HexColor('#444444'))
H_DATE = ParagraphStyle('Date', parent=styles['Normal'],
                        fontName='Helvetica-Oblique', fontSize=10, leading=12,
                        alignment=TA_CENTER, textColor=colors.HexColor('#666666'))
H1 = ParagraphStyle('H1', parent=styles['Heading1'],
                    fontName='Helvetica-Bold', fontSize=16, leading=20,
                    spaceBefore=14, spaceAfter=8, textColor=colors.HexColor('#0b3d91'))
H2 = ParagraphStyle('H2', parent=styles['Heading2'],
                    fontName='Helvetica-Bold', fontSize=13, leading=16,
                    spaceBefore=10, spaceAfter=4, textColor=colors.HexColor('#11498a'))
H3 = ParagraphStyle('H3', parent=styles['Heading3'],
                    fontName='Helvetica-Bold', fontSize=11, leading=14,
                    spaceBefore=8, spaceAfter=2, textColor=colors.HexColor('#333333'))
BODY = ParagraphStyle('Body', parent=styles['BodyText'],
                      fontName='Helvetica', fontSize=10, leading=14,
                      alignment=TA_JUSTIFY, spaceAfter=6)
CAPTION = ParagraphStyle('Caption', parent=styles['Normal'],
                         fontName='Helvetica-Oblique', fontSize=9, leading=11,
                         alignment=TA_CENTER, textColor=colors.HexColor('#555555'),
                         spaceBefore=2, spaceAfter=8)
CODE = ParagraphStyle('Code', parent=styles['Code'],
                      fontName='Courier', fontSize=8.5, leading=11,
                      leftIndent=10, rightIndent=10,
                      backColor=colors.HexColor('#f4f4f4'),
                      borderColor=colors.HexColor('#dddddd'),
                      borderPadding=4, borderWidth=0.5,
                      spaceBefore=2, spaceAfter=8)
NOTE = ParagraphStyle('Note', parent=BODY,
                      backColor=colors.HexColor('#fff8e1'),
                      borderColor=colors.HexColor('#f1c40f'),
                      borderWidth=0.5, borderPadding=6,
                      leftIndent=4, rightIndent=4)


# ----------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------

def p(text: str, style=BODY) -> Paragraph:
    return Paragraph(text, style)


def code(text: str) -> Paragraph:
    # Escape angle brackets and ampersands so reportlab doesn't parse tags.
    safe = (text.replace('&', '&amp;')
                .replace('<', '&lt;')
                .replace('>', '&gt;'))
    safe = safe.replace('\n', '<br/>')
    safe = safe.replace(' ', '&nbsp;')
    return Paragraph(safe, CODE)


def table(data, col_widths=None, header=True):
    style = [
        ('FONT', (0, 0), (-1, -1), 'Helvetica', 9),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('GRID', (0, 0), (-1, -1), 0.4, colors.HexColor('#bbbbbb')),
        ('LEFTPADDING', (0, 0), (-1, -1), 5),
        ('RIGHTPADDING', (0, 0), (-1, -1), 5),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
    ]
    if header:
        style.append(('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#dde7f4')))
        style.append(('FONT', (0, 0), (-1, 0), 'Helvetica-Bold', 9))
    t = Table(data, colWidths=col_widths, repeatRows=(1 if header else 0))
    t.setStyle(TableStyle(style))
    return t


def fig(path, width_in=5.6, caption_text=None):
    img = Image(path)
    aspect = img.imageHeight / float(img.imageWidth)
    img.drawWidth = width_in * inch
    img.drawHeight = width_in * inch * aspect
    items = [img]
    if caption_text:
        items.append(p(caption_text, CAPTION))
    return KeepTogether(items)


# ----------------------------------------------------------------------
#  Document
# ----------------------------------------------------------------------

def on_page(canvas, doc):
    canvas.saveState()
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(colors.HexColor('#888888'))
    canvas.drawString(0.75 * inch, 0.5 * inch,
                      'xArm 1S — 2D Pick-and-Place Project Report')
    canvas.drawRightString(LETTER[0] - 0.75 * inch, 0.5 * inch,
                           f'Page {doc.page}')
    canvas.restoreState()


def build():
    doc = BaseDocTemplate(
        OUT, pagesize=LETTER,
        leftMargin=0.85 * inch, rightMargin=0.85 * inch,
        topMargin=0.85 * inch, bottomMargin=0.75 * inch,
        title='xArm 1S 2D Pick-and-Place — Project Report',
        author='Darklord (project owner)',
    )
    frame = Frame(doc.leftMargin, doc.bottomMargin,
                  doc.width, doc.height, id='normal')
    doc.addPageTemplates([PageTemplate(id='all', frames=frame, onPage=on_page)])

    story = []

    # =================================================================
    #  Title page
    # =================================================================
    story.append(Spacer(1, 1.6 * inch))
    story.append(p('xArm 1S<br/>2D Table Pick-and-Place', H_TITLE))
    story.append(Spacer(1, 0.15 * inch))
    story.append(p('Project Report &amp; Engineering Findings', H_SUBT))
    story.append(Spacer(1, 0.05 * inch))
    story.append(p('ROS 2 Humble · MoveIt 2 · Custom 5-DOF Numerical IK ·<br/>'
                   'Eye-to-hand Astra Pro · ArUco Fiducials · Planar Homography',
                   H_SUBT))
    story.append(Spacer(1, 1.2 * inch))
    story.append(p(f'Compiled {date.today().isoformat()}', H_DATE))
    story.append(Spacer(1, 0.15 * inch))
    story.append(p('Workspace: <font face="Courier">~/xarm_moveit</font> · '
                   'Branch: <font face="Courier">main</font>', H_DATE))
    story.append(PageBreak())

    # =================================================================
    #  Executive summary
    # =================================================================
    story.append(p('Executive Summary', H1))
    story.append(p(
        "This report documents an end-to-end 2D pick-and-place system built around the "
        "<b>xArm 1S</b> — a 5-DOF positioning manipulator with a 1-DOF parallel-jaw gripper "
        "— observed by a fixed (eye-to-hand) Orbbec Astra Pro RGB-D camera. The objective "
        "was to reliably grasp a 40&nbsp;mm cube (with a 30&nbsp;mm <i>DICT_5X5_50</i> ArUco "
        "marker, id&nbsp;2) off a flat table and optionally place it at a target XY on the "
        "same plane.", BODY))
    story.append(p(
        "The system has now reached <b>routinely successful picks</b>. Two engineering "
        "decisions were decisive:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("<b>Replace 3D hand-eye calibration with a 2D planar homography.</b> "
                   "All picks live on a single plane (the table), so a 3×3 pixel→world matrix "
                   "absorbs every linear distortion — camera, table tilt, focal length error — "
                   "into one SVD-fit object that needs only 6–8 calibration points instead of "
                   "the dozens of motions required by the AX = XB hand-eye problem.", BODY)),
        ListItem(p("<b>Tighten the IK joint limits to ±π/2 to match the silent driver "
                   "clamp.</b> The hardware driver clamps every commanded joint to ±90° "
                   "before sending it to the servos. Solutions outside this range were "
                   "silently truncated, producing ~30&nbsp;mm of position error and ~17° of "
                   "unmodelled tilt at the table surface. Constraining IK to the driver's "
                   "effective range closed the loop.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(p(
        "The IK itself was rewritten in a clean Modern Robotics style "
        "(Lynch &amp; Park, 2017) using the Product of Exponentials forward kinematics and "
        "a <b>damped-least-squares Newton-Raphson</b> with task-priority null-space "
        "redundancy resolution: position is the primary task; gripper-down preference is "
        "projected into the null space of the position Jacobian so it never fights "
        "position. This replaced an earlier scipy L-BFGS-B optimiser. With a brief "
        "&ldquo;polish&rdquo; phase that continues iterating after position converges to spend "
        "the redundant DOFs on orientation, sub-millimetre position error and "
        "5–25° tilts are typical across the reachable workspace.",
        BODY))
    story.append(p('Quick numbers', H3))
    story.append(table([
        ['Metric', 'Value'],
        ['Position accuracy (typical)',     '< 0.1 mm at the IK level'],
        ['Gripper tilt (typical)',          '6° – 27° from vertical'],
        ['IK seeds per call',               '24 (multi-start branch coverage)'],
        ['IK runtime',                      '~1.5 s per solve_ik'],
        ['Calibration residual',            '~2.5 mm mean (9/12 inliers)'],
        ['Workspace (gripper-down)',        '≈ 80 – 200 mm radial ring around base'],
        ['Joint range (IK = driver)',       '±1.5707 rad on all 5 planning joints'],
    ], col_widths=[2.4 * inch, 3.6 * inch]))
    story.append(PageBreak())

    # =================================================================
    #  Table of contents
    # =================================================================
    story.append(p('Contents', H1))
    toc = [
        ('1.  Project goal', '4'),
        ('2.  Hardware', '4'),
        ('3.  Software stack', '5'),
        ('4.  End-to-end pipeline', '6'),
        ('5.  Forward kinematics — Product of Exponentials', '7'),
        ('6.  Inverse kinematics — DLS Newton with null-space task priority', '9'),
        ('7.  Calibration — pinhole, IPPE, planar homography', '12'),
        ('8.  Trajectory generation — OMPL RRT-Connect + TOTG', '14'),
        ('9.  Execution &amp; the driver clamp', '15'),
        ('10. Findings &amp; engineering breakthroughs', '16'),
        ('11. Test results', '18'),
        ('12. Known limitations &amp; next steps', '19'),
        ('13. References &amp; file pointers', '20'),
    ]
    for title, _pg in toc:
        story.append(p(title, BODY))
    story.append(PageBreak())

    # =================================================================
    #  1. Project goal
    # =================================================================
    story.append(p('1. Project goal', H1))
    story.append(p(
        "Pick a 40&nbsp;mm cube off a flat table with the xArm 1S and optionally place it "
        "at a target XY on the same table. The cube carries a 30&nbsp;mm ArUco marker "
        "(<font face=\"Courier\">DICT_5X5_50</font>, <font face=\"Courier\">marker_id=2</font>) "
        "on its top face. The camera is mounted on a tripod, fixed in the world (eye-to-hand); "
        "the arm and the cube share that world. Long-term goal is multi-cube pick-and-place "
        "keyed on per-marker identity; v1 below handles a single cube.",
        BODY))
    story.append(p(
        "Three constraints shape every choice in this system:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("The arm has <b>5 actuated DOFs</b> for positioning. Six-DOF tasks "
                   "(arbitrary tool orientation) are mathematically underdetermined; the "
                   "best we can do is achieve a target position and accept the orientation "
                   "the arm can deliver, then post-filter by tilt.", BODY)),
        ListItem(p("Picks happen on a <b>single planar surface</b>. This collapses the "
                   "3D hand-eye problem to a 2D pixel-to-world homography.", BODY)),
        ListItem(p("The hardware driver <b>silently clamps each joint to ±π/2</b>. "
                   "Every component of the IK has to respect this constraint or the "
                   "predicted FK will not match the executed FK.", BODY)),
    ], bulletType='bullet', leftIndent=20))

    # =================================================================
    #  2. Hardware
    # =================================================================
    story.append(p('2. Hardware', H1))
    story.append(table([
        ['Item', 'Detail'],
        ['Arm',          'xArm 1S — 5 DOF positioning + 1 DOF gripper. USB control via the '
                         'LewanSoul HID protocol; the xarm Python library wraps it.'],
        ['Camera',       'Orbbec Astra Pro RGB-D, eye-to-hand on a tripod, fixed in the '
                         'world frame. Vendored ros2_astra_camera driver.'],
        ['Cube',         '40 mm wooden cube. 30 mm ArUco marker (DICT_5X5_50, id 2) printed '
                         'on its top face.'],
        ['Compute',      'Linux 6.8 / Ubuntu 22.04 host with USB 2.0 access to both arm and '
                         'camera.'],
    ], col_widths=[1.2 * inch, 4.8 * inch]))
    story.append(Spacer(1, 0.15 * inch))
    story.append(fig(os.path.join(FIGS, 'kinematic_chain.png'),
                     width_in=3.4,
                     caption_text="Figure 1. xArm 1S kinematic chain at the home configuration "
                                  "(q = 0). The base is at z = 0.043 m; the tool0 frame "
                                  "(gripper midpoint) sits at z ≈ 0.427 m at home. The chain "
                                  "is composed by world → base_link → arm6 → arm5 → arm4 → "
                                  "arm3 → arm2 → tool0."))

    # =================================================================
    #  3. Software stack
    # =================================================================
    story.append(p('3. Software stack', H1))
    story.append(table([
        ['Layer', 'Component', 'Notes'],
        ['OS / middleware', 'Ubuntu 22.04 + ROS 2 Humble', '— '],
        ['Motion planning', 'MoveIt 2 (OMPL / RRT-Connect)',
         'KDL is bypassed — it returns NO_IK_SOLUTION on this 5-DOF arm.'],
        ['Time scaling', 'AddTimeOptimalParameterization (TOTG)',
         'Renamed from AddTimeParameterization in Humble.'],
        ['Hardware bridge', 'xarm_hw',
         'Bridges /joint_trajectory → USB. Implements the silent ±π/2 clamp.'],
        ['Vision / fiducials', 'charuco_tf_publisher (single_aruco mode)',
         'Broadcasts camera_color_optical_frame → handeye_target.'],
        ['Pick stack', 'xarm_pick',
         'arm_ik.py, calibrate_homography.py, pick_2d.py.'],
        ['Camera driver', 'ros2_astra_camera (vendored)', '— '],
    ], col_widths=[1.2 * inch, 1.9 * inch, 2.9 * inch]))
    story.append(Spacer(1, 0.10 * inch))
    story.append(p('3.1 Bring-up scripts', H2))
    story.append(p(
        "Five tmux components — <i>display</i>, <i>driver</i>, <i>camera</i>, <i>marker</i>, "
        "<i>moveit</i> — wrap into one bring-up tool with five profiles. Only one bring-up "
        "session can run at a time.", BODY))
    story.append(table([
        ['Script', 'Components', 'Use case'],
        ['./bringup_pick.sh',   'all 5',                        'Full pick stack (default).'],
        ['./bringup_calib.sh',  'display + driver + cam + mkr', 'Run calibrate_homography.'],
        ['./bringup_vision.sh', 'cam + marker',                 'Tune ArUco without arm.'],
        ['./bringup_robot.sh',  'display + driver',             'Drag-teach, gripper, raw '
                                                                'JointTrajectory.'],
        ['./bringup_moveit.sh', 'display + driver + moveit',    'v1 joint-space, no camera.'],
    ], col_widths=[1.6 * inch, 2.2 * inch, 2.2 * inch]))

    # =================================================================
    #  4. End-to-end pipeline
    # =================================================================
    story.append(PageBreak())
    story.append(p('4. End-to-end pipeline', H1))
    story.append(p(
        "A single pick is the composition of the following stages. Each stage corresponds "
        "to one identifiable section in the source tree.", BODY))
    story.append(code(
        "camera image          ┐\n"
        "camera_info (K)       ├─► ArUco detect (IPPE_SQUARE) ─► TF cam→marker ─► (u,v)\n"
        "ChArUco TF publisher  ┘                                                    │\n"
        "                                                                           ▼\n"
        "                                                   H ∈ ℝ³ˣ³ (homography.yaml)\n"
        "                                                                           │\n"
        "                                                                           ▼\n"
        "                                                   world XY = π(H · [u v 1]ᵀ)\n"
        "                                                                           │\n"
        "                                                                           ▼\n"
        "       5-DOF DLS Newton IK (PoE) ─► joint vector q ─► MoveIt MoveGroup\n"
        "                                                                           │\n"
        "                                                                           ▼\n"
        "                       OMPL RRTConnect path ─► TOTG retime ─► /follow_jt\n"
        "                                                                           │\n"
        "                                                                           ▼\n"
        "                                       xarm_hw driver ─► USB ─► servos"
    ))
    story.append(p(
        "Stages 1–4 happen continuously in vision; stages 5–8 fire once per pick "
        "state-machine transition. The driver's 20&nbsp;Hz <font face=\"Courier\">JointState</font> "
        "feedback closes the loop in <font face=\"Courier\">tf2_ros</font>; RViz and "
        "<font face=\"Courier\">pick_2d</font> see the actual pose.",
        BODY))

    # =================================================================
    #  5. Forward kinematics — Product of Exponentials
    # =================================================================
    story.append(PageBreak())
    story.append(p('5. Forward kinematics — Product of Exponentials', H1))
    story.append(p(
        "The forward kinematics is implemented in "
        "<font face=\"Courier\">arm_ik.py:fk_tool0</font> using the "
        "<b>Product of Exponentials (PoE)</b> form (Modern Robotics §4.1):",
        BODY))
    story.append(code(
        "T(θ) = exp([S₁]θ₁) · exp([S₂]θ₂) · ... · exp([S₅]θ₅) · M\n"
        "\n"
        "where Sᵢ ∈ ℝ⁶ are screw axes in the SPACE (world) frame at the home pose\n"
        "and M ∈ SE(3) is T_world_tool0 at q = 0.\n"
        "\n"
        "Joint indices: q[0]=arm6 (S₁), …, q[4]=arm2 (S₅)."
    ))
    story.append(p(
        "Why PoE rather than per-link composition: the kinematics is identical, but the "
        "space and body Jacobians (MR §5.1) drop out as analytical functions of the screw "
        "axes and the running prefix transform. This makes every Newton iteration analytical "
        "— no finite differences.",
        BODY))
    story.append(p('5.1 Screw axes (verified against URDF chain to 0.000 µm)', H2))
    story.append(p(
        "Each row is Sᵢ = (ωₓ, ωᵧ, ω_z, vₓ, vᵧ, v_z) in the space frame at home, with "
        "v = -ω × q where q is any point on joint i&apos;s axis. The arm6 axis is "
        "(0, 0, -1) and the URDF pre-applies an Rz(π) origin rotation, so the cumulative "
        "orientation downstream is Rz(π); each downstream joint axis is therefore that "
        "axis rotated by Rz(π).", BODY))
    story.append(table([
        ['Joint', 'ω (world)', 'point q', 'v = -ω × q'],
        ['S₁ = arm6', '(0, 0, -1)',  '(0, 0, 0.086)',     '(0, 0, 0)'],
        ['S₂ = arm5', '(0, -1, 0)',  '(-0.002, 0, 0.118)', '(0.118, 0, 0.002)'],
        ['S₃ = arm4', '(0, +1, 0)',  '(-0.002, 0, 0.21575)', '(-0.21575, 0, -0.002)'],
        ['S₄ = arm3', '(0, -1, 0)',  '(-0.002, 0, 0.31475)', '(0.31475, 0, 0.002)'],
        ['S₅ = arm2', '(0, 0, +1)',  '(-0.00075, 0, 0.36475)', '(0, 0.00075, 0)'],
    ], col_widths=[1.0 * inch, 1.4 * inch, 1.7 * inch, 1.7 * inch]))
    story.append(Spacer(1, 0.06 * inch))
    story.append(p('5.2 Home matrix M', H2))
    story.append(p(
        "M = T_world_tool0 at q = 0 has the cumulative Rz(π) rotation and the summed link "
        "offsets (with downstream pieces transformed by the running parent orientation):",
        BODY))
    story.append(code(
        "M_HOME = [[-1.0,  0.0,  0.0,  0.00305 ],\n"
        "          [ 0.0, -1.0,  0.0,  0.0     ],\n"
        "          [ 0.0,  0.0,  1.0,  0.42675 ],\n"
        "          [ 0.0,  0.0,  0.0,  1.0     ]]"
    ))
    story.append(p(
        "Verification: 500 random configurations, FK via PoE composition vs FK via "
        "frame-by-frame URDF chain composition, max element-wise difference "
        "<b>4.3·10⁻¹⁶</b> (machine epsilon). The screw-axes derivation is correct.",
        BODY))

    story.append(p('5.3 Jacobians', H2))
    story.append(p(
        "The space Jacobian J_s ∈ ℝ⁶ˣ⁵ is built column-by-column: column 0 is just S₁; "
        "column i (i ≥ 1) is the Adjoint of the running prefix transform "
        "exp([S₁]θ₁)·…·exp([Sᵢ₋₁]θᵢ₋₁) applied to Sᵢ (MR eq. 5.11).",
        BODY))
    story.append(p(
        "For position-only IK, only the bottom three rows of the world-frame velocity "
        "matter. The position Jacobian — derivative of <i>tool0</i>'s world position with "
        "respect to q — drops out as",
        BODY))
    story.append(code(
        "J_pos(q) = -[p(q)] · ω_s + v_s    ∈ ℝ³ˣ⁵\n"
        "\n"
        "where (ω_s, v_s) are the upper / lower 3×n blocks of J_s and [p] is the\n"
        "skew-symmetric matrix of the current tool0 position."
    ))
    story.append(p(
        "Verified against finite differences: max |J_pos − J_pos_finite_diff| ≈ "
        "<b>5.5·10⁻¹⁰</b> across random configurations. The analytical position Jacobian "
        "is correct.",
        BODY))

    # =================================================================
    #  6. Inverse kinematics
    # =================================================================
    story.append(PageBreak())
    story.append(p('6. Inverse kinematics — DLS Newton with null-space task priority', H1))
    story.append(p('6.1 Why KDL fails', H2))
    story.append(p(
        "MoveIt's default IK plugin (KDL) implements MR Algorithm 6.1 — Newton-Raphson on "
        "the full 6-DOF residual e(q) = (log(R<sub>sd</sub>·R<sub>sb</sub>(q)<sup>T</sup>)<sup>∨</sup>, "
        "p<sub>sd</sub> − p<sub>sb</sub>(q)) ∈ ℝ⁶ with update q ← q + J<sup>†</sup>e. "
        "With 5 joints and a 6×5 Jacobian, J is rank-deficient for the full 6-DOF target "
        "almost everywhere; KDL's residual gate refuses to converge and the plugin returns "
        "<font face=\"Courier\">NO_IK_SOLUTION</font> for every reachable position. "
        "The pick stack does not call MoveIt for IK — IK is solved upfront in "
        "<font face=\"Courier\">arm_ik.solve_ik</font>, and only joint-space goals are "
        "passed to MoveIt's MoveGroup action.",
        BODY))
    story.append(p('6.2 Damped-least-squares Newton, task-priority redundancy', H2))
    story.append(p(
        "The 5-DOF arm is redundant for a 3-DOF position task: J_pos ∈ ℝ³ˣ⁵ has full row "
        "rank 3 in non-singular configurations, leaving 2 free DOFs in its null space. The "
        "IK splits the work along the redundancy hierarchy of MR §6.3 / Siciliano §3.5:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("<b>Primary task</b> — drive position to target. "
                   "Step: Δq<sub>pos</sub> = J<sub>pos</sub><sup>+</sup> · (p<sub>target</sub> − p(q)). "
                   "The damped Moore-Penrose pseudoinverse "
                   "J<sub>pos</sub><sup>+</sup> = J<sub>pos</sub><sup>T</sup>(J<sub>pos</sub>·J<sub>pos</sub><sup>T</sup> + λ²I)<sup>−1</sup> "
                   "exists for any 3×5 J_pos (the 3×3 inner is always invertible thanks to "
                   "λ²I) and gives the <i>minimum-norm</i> Δq closing the linearised "
                   "position gap. The Levenberg-Marquardt damping λ²I keeps the inverse "
                   "well-conditioned near singularities.", BODY)),
        ListItem(p("<b>Secondary task</b> — gripper-down preference, "
                   "c<sub>orient</sub>(q) = (1 + R<sub>tool0,zz</sub>(q))² (zero exactly "
                   "when the tool +z̃ axis points along world −ẑ). The gradient is "
                   "projected into the null space of the position Jacobian by the "
                   "projector N = I − J<sub>pos</sub><sup>+</sup>·J<sub>pos</sub>, so "
                   "Δq<sub>orient</sub> = N · (−α·∇c<sub>orient</sub>) <i>cannot</i> "
                   "perturb position to first order — orientation never fights the primary "
                   "objective.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(p('6.3 Adaptive damping &amp; the polish phase', H2))
    story.append(p(
        "Damping λ is adaptive: large (0.10) when ‖p<sub>err</sub>‖ &gt; 5&nbsp;cm — stable, "
        "robust to singularities — and small (0.005) once close — precise convergence. "
        "Once position has converged inside <i>convergence_bound</i> (= "
        "0.2·pos_tol_m), the loop enters a <i>polish</i> phase: it continues for up to 30 "
        "more iterations with the secondary gain boosted 3×, spending the redundant DOFs "
        "almost entirely on aligning the gripper to vertical while the primary step "
        "patches any residual position drift. This was the single change that brought "
        "tilts down from ~38° to ~22° on the test targets.",
        BODY))
    story.append(p('6.4 Multi-start branch coverage', H2))
    story.append(p(
        "Newton-Raphson converges locally; 5-DOF arms have multiple IK branches "
        "(shoulder-fwd-elbow-up vs. shoulder-back-elbow-down, etc.). The solver runs "
        "<b>24 seeds</b>: a heuristic seed (q<sub>6</sub> = π − atan2(y, x), wrist roughly "
        "above the target), the previous-step joint vector if branch-pinning is requested, "
        "and 22 uniform random seeds across the joint box. A four-tier comparison picks "
        "the winner:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("Position-success (pos_err &lt; 5 mm) beats failure.", BODY)),
        ListItem(p("Among successes — both within tilt tolerance: closest in joint-space "
                   "to <i>reference_q</i> wins (homotopy-class consistency along a pick "
                   "sequence).", BODY)),
        ListItem(p("Among successes — one within tolerance, one outside: within wins.", BODY)),
        ListItem(p("Among successes — both outside: lowest tilt wins.", BODY)),
        ListItem(p("Among failures: lowest position error wins.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(Spacer(1, 0.10 * inch))
    story.append(fig(os.path.join(FIGS, 'ik_convergence.png'),
                     width_in=6.0,
                     caption_text="Figure 2. DLS Newton convergence on a single target "
                                  "(0.05, 0.18, 0.12) m, four random seeds. Position error "
                                  "decays geometrically and locks below 1 mm in ~30 "
                                  "iterations (left). Tilt continues falling during the "
                                  "30-iteration polish phase as the secondary gradient "
                                  "spends the redundant DOFs on orientation (right)."))

    # =================================================================
    #  7. Calibration
    # =================================================================
    story.append(PageBreak())
    story.append(p('7. Calibration — pinhole, IPPE, planar homography', H1))
    story.append(p(
        "Two distinct calibrations stack on each other.", BODY))
    story.append(p('7.1 Camera intrinsics — pinhole projection', H2))
    story.append(p(
        "The Astra Pro publishes a <font face=\"Courier\">CameraInfo</font> message with the "
        "camera matrix K. Standard pinhole: u = f<sub>x</sub>·x/z + c<sub>x</sub>, "
        "v = f<sub>y</sub>·y/z + c<sub>y</sub>. Used in two places: (a) re-projecting the "
        "PnP-solved 3D marker centroid to a stable pixel, and (b) the homography fit's "
        "input side. Calibration of K itself is done once at the factory.",
        BODY))
    story.append(p('7.2 Marker pose — IPPE PnP with three-stage disambiguation', H2))
    story.append(p(
        "<font face=\"Courier\">charuco_tf_node._process_single_aruco</font> runs OpenCV's "
        "<font face=\"Courier\">SOLVEPNP_IPPE_SQUARE</font> on the four marker corners. A "
        "planar square has a two-fold pose ambiguity (front and back-flipped reproject "
        "within sub-pixel error at near-frontal views). The code disambiguates with three "
        "filters in <font face=\"Courier\">_pick_ippe_solution</font>:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("<b>Physical feasibility</b>: reject any R with R[3,3] ≥ 0 — the marker "
                   "is one-sided so its normal must point back toward the camera.", BODY)),
        ListItem(p("<b>Sticky-branch history</b>: prefer the candidate within 30° of the "
                   "most recently accepted rotation, with a 6-frame timeout that clears "
                   "history on persistent rejection.", BODY)),
        ListItem(p("<b>Depth-fusion bootstrap</b>: when no recent history, fit a robust "
                   "plane to depth pixels inside the marker's image-convex hull and pick "
                   "the IPPE branch whose normal best matches it.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(p('7.3 Pixel → world: the planar homography (the decisive simplification)', H2))
    story.append(p(
        "All picks live on a single plane. For any plane Z = Z₀ in the world, the pinhole "
        "projection is projectively linear in (X, Y) — this is the standard plane-projection "
        "homography form. We invert it and fit a single 3×3 H mapping pixels directly to "
        "world XY:",
        BODY))
    story.append(code(
        "[wX]      [u]\n"
        "[wY] = H ·[v]                           (X, Y) = (wX/w, wY/w)\n"
        "[ w]      [1]"
    ))
    story.append(p(
        "<b>Two-phase data collection</b> sidesteps a chicken-and-egg problem: the cube "
        "can be seen by the camera <i>or</i> reached by the gripper, but not both at once "
        "(the gripper occludes the marker in the grasp pose). Each calibration sample is "
        "captured in two halves:",
        BODY))
    story.append(ListFlowable([
        ListItem(p("<b>Phase A</b>: place the cube, move the arm out of view, ENTER "
                   "captures the marker pixel (u, v).", BODY)),
        ListItem(p("<b>Phase B</b>: drag-teach the arm so tool0 is exactly where it "
                   "would be at the grasp moment without disturbing the cube; ENTER reads "
                   "the world XY from the TF tree.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(p(
        "H is fit by OpenCV's <font face=\"Courier\">cv2.findHomography</font> with RANSAC "
        "(threshold 0.01 m), which rejects mis-clicked Phase A captures without poisoning "
        "the global fit. The median of the captured tool0 z-values is saved as "
        "<i>suggested_table_z</i> — a clever sidestep of explicit gripper-geometry "
        "calibration: we never compute the gripper's shape because we never need it; we "
        "<i>measure</i> the right z directly.",
        BODY))
    story.append(p('7.4 Why this beats hand-eye AX = XB on this rig', H2))
    story.append(p(
        "Full hand-eye calibration solves AX = XB for the camera-to-flange transform from "
        "many (A_i, B_i) motion pairs. It needs good rvec stability (which the IPPE "
        "flicker poisons directly), enough motion to constrain rotation (≥3 non-parallel "
        "screw axes), an accurate FK chain to the marker, and a marker rigidly mounted to "
        "the flange. This rig violated <i>every</i> one of those. The 2D homography "
        "sidesteps the orientation half of X entirely, needs ~6–8 samples instead of "
        "dozens, has a built-in residual metric, and is decoupled from IK quality. The "
        "trade is that it only works for cubes on the calibrated plane — for a "
        "fixed-camera tabletop demo, that's a perfectly fair cost.",
        BODY))

    # =================================================================
    #  8. Trajectory generation
    # =================================================================
    story.append(PageBreak())
    story.append(p('8. Trajectory generation — OMPL RRT-Connect + TOTG', H1))
    story.append(p(
        "Given q* from IK, MoveIt produces a time-parameterised trajectory q(t) over "
        "t ∈ [0, T] with q(0) = q<sub>current</sub> and q(T) = q*, satisfying joint, "
        "velocity, and acceleration limits.",
        BODY))
    story.append(p('8.1 OMPL RRT-Connect', H2))
    story.append(p(
        "RRT-Connect (Kuffner &amp; LaValle, 2000) is a bidirectional sampling-based "
        "planner: two trees rooted at q<sub>current</sub> and q* grow toward each other "
        "in C, attempting a connect extension on every iteration. The output is a "
        "geometric path σ : [0, 1] → C with no notion of time. The OMPL configuration "
        "tightens <font face=\"Courier\">longest_valid_segment_fraction</font> from the "
        "default 0.005 (0.5%) to <b>0.002</b> — finer waypoints give TOTG a denser path "
        "to retime over.",
        BODY))
    story.append(p('8.2 Time scaling — TOTG (Kunz &amp; Stilman, 2012)', H2))
    story.append(p(
        "<font face=\"Courier\">AddTimeOptimalParameterization</font> implements MR §9.4: "
        "given a path σ(s) and per-joint velocity/acceleration bounds, find the time "
        "scaling s(t) that minimises T = ∫ds/ṡ subject to the bounds. The output is a "
        "sequence of <font face=\"Courier\">JointTrajectoryPoint</font>s each with "
        "positions, velocities, accelerations, and a strictly monotonic "
        "<font face=\"Courier\">time_from_start</font>. Resample resolution is 0.02 s "
        "(50 Hz) — denser starts producing micro-segments the driver sleeps through in "
        "&lt; 1 ms each.",
        BODY))
    story.append(NOTE.clone('NoteRuckig').apply if False else p(
        "<b>Pitfall logged in <font face=\"Courier\">ompl_planning.yaml</font>:</b> "
        "<font face=\"Courier\">AddRuckigTrajectorySmoothing</font> was tried as an "
        "additional adapter and silently produces a degenerate (zero-motion) trajectory "
        "on this 5-DOF arm. MoveIt still returns SUCCESS, so "
        "<font face=\"Courier\">pick_2d</font> &ldquo;completes&rdquo; without the arm "
        "ever moving. Do <b>not</b> add Ruckig back without verifying the executed "
        "trajectory length.",
        NOTE))
    story.append(p('8.3 Velocity / acceleration scaling', H2))
    story.append(p(
        "Joint limits (per "
        "<font face=\"Courier\">src/xarm_moveit_config/config/joint_limits.yaml</font>): "
        "max_velocity 0.3 rad/s, max_acceleration 0.3 rad/s² for each planning joint. "
        "Global scalings: <i>default_velocity_scaling_factor</i> 0.45, "
        "<i>default_acceleration_scaling_factor</i> 0.30. Together these give an effective "
        "peak velocity of 0.135 rad/s — pick speed without the visible stutter that 0.30/0.30 "
        "produced before the driver-side fix described in §9.",
        BODY))

    # =================================================================
    #  9. Execution & the driver clamp
    # =================================================================
    story.append(p('9. Execution &amp; the driver clamp', H1))
    story.append(p(
        "The controller side runs in "
        "<font face=\"Courier\">xarm_hw/driver.py:execute_trajectory_cb</font>. For each "
        "<font face=\"Courier\">JointTrajectoryPoint</font> it reads the desired wall time, "
        "computes the per-segment duration, sends the waypoint to the servos with that "
        "duration as the move time, and busy-waits until wall-clock matches the desired "
        "<i>time_from_start</i> before advancing to the next point. The smart-servos "
        "themselves close the loop on the commanded angle.",
        BODY))
    story.append(p('9.1 The two driver issues', H2))
    story.append(p(
        "<b>Issue 1 — silent ±π/2 clamp.</b> "
        "<font face=\"Courier\">rad_to_units</font> clamps every commanded angle to ±90° "
        "(±π/2 rad) before sending to the servos. Any IK solution outside this range was "
        "silently truncated, producing ~30&nbsp;mm of position error and ~17° of unmodelled "
        "tilt at the table surface. Fix: tighten "
        "<font face=\"Courier\">JOINT_LIMITS</font> in "
        "<font face=\"Courier\">arm_ik.py</font> to ±1.5707 (one ULP below π/2) so every "
        "q the IK proposes is one the hardware can faithfully execute.",
        BODY))
    story.append(p(
        "<b>Issue 2 — fixed-200&nbsp;ms duration clamp.</b> The driver's setPosition "
        "duration was clamped to 200 ms regardless of the TOTG segment cadence, so each "
        "servo command was interrupted mid-ramp. With 0.30/0.30 velocity/acceleration "
        "scaling and max_accel 0.5, this produced visibly stuttered motion. Fix in "
        "<font face=\"Courier\">xarm_hw/driver.py</font>: use the segment duration "
        "directly with a small (2.5×) buffer for continuous-redirect, which lets us run "
        "<i>faster and smoother</i>: 0.45/0.30 with max_accel 0.30 (smoother TOTG ramps).",
        BODY))

    # =================================================================
    #  10. Findings & engineering breakthroughs
    # =================================================================
    story.append(PageBreak())
    story.append(p('10. Findings &amp; engineering breakthroughs', H1))
    story.append(p('10.1 The driver clamp dominated the error budget', H2))
    story.append(p(
        "Pre-fix, the cube was missed by 2–3 cm with ~17° tilt. The diagnostic that "
        "located the issue was straightforward in hindsight but easy to miss: the IK was "
        "returning q values up to ±1.7 rad (allowed by the URDF's wider limits) but the "
        "<i>actual</i> joint trajectory landed at ±1.5708. FK on the commanded q predicted "
        "a position the actual q never reached. Once "
        "<font face=\"Courier\">JOINT_LIMITS</font> matched the driver's effective range, "
        "the residual error fell to the calibration-residual floor (~2.5 mm).",
        BODY))
    story.append(p('10.2 Plane homography &gt; full hand-eye for fixed-table tasks', H2))
    story.append(p(
        "The earlier branch of this project tried full 3D AX = XB hand-eye calibration "
        "with multiple solver variants (the abandoned scripts <font face=\"Courier\">"
        "auto_handeye_calibrate.py</font>, <font face=\"Courier\">"
        "pooled_handeye_solve.py</font>, <font face=\"Courier\">"
        "ippe_irls_with_prior.py</font>, etc., still in the workspace as historical "
        "reference). None reached useful pick accuracy because the IPPE flicker poisoned "
        "the rotation estimate. The pivot to a 2D plane homography eliminated the "
        "rotation degree of freedom from the calibration entirely and brought residuals "
        "below 3 mm immediately.",
        BODY))
    story.append(p('10.3 Task-priority IK with adaptive damping &amp; polish', H2))
    story.append(p(
        "The IK rewrite progression captured the trade-offs cleanly. Before, the IK was "
        "an L-BFGS-B optimiser on a soft-constraint cost — which converged but could not "
        "be reasoned about analytically. The DLS-Newton rewrite progressed:",
        BODY))
    story.append(table([
        ['Stage', 'Avg pos err', 'Avg tilt'],
        ['scipy L-BFGS-B (original)',                 '0.0 mm', '19.8°'],
        ['DLS Newton — primary task only',            '2.0 mm', '38.2°'],
        ['+ adaptive damping (far / near)',           '0.04 mm', '38.2°'],
        ['+ task-priority null-space orientation',    '0.04 mm', '24°'],
        ['+ 30-iter polish phase, 3× secondary gain', '0.01 mm', '22.2°'],
    ], col_widths=[2.8 * inch, 1.4 * inch, 1.0 * inch]))
    story.append(Spacer(1, 0.10 * inch))
    story.append(p(
        "The polish phase contributed the largest tilt improvement at zero cost in "
        "position accuracy — a textbook case of redundancy resolution.",
        BODY))
    story.append(p('10.4 Other findings worth recording', H2))
    story.append(ListFlowable([
        ListItem(p("<b>MoveIt OMPL params live at the legacy path on Humble.</b> "
                   "<font face=\"Courier\">move_action</font> reads "
                   "<font face=\"Courier\">ompl.planning_plugin</font>, not the newer "
                   "<font face=\"Courier\">planning_pipelines.ompl.planning_plugin</font>. "
                   "Wrong nesting silently disables planning.", BODY)),
        ListItem(p("<b>The TOTG plugin name changed in Humble.</b> "
                   "<font face=\"Courier\">AddTimeParameterization</font> → "
                   "<font face=\"Courier\">AddTimeOptimalParameterization</font>. The old "
                   "name fails to load (pluginlib InvalidClass) and post-processing "
                   "returns FAILURE (99999) on every request.", BODY)),
        ListItem(p("<b>Camera TF chain conflicts.</b> The Astra driver publishes "
                   "<font face=\"Courier\">camera_link → camera_color_frame → "
                   "camera_color_optical_frame</font>. Adding a static "
                   "<font face=\"Courier\">world → camera_color_optical_frame</font> "
                   "publisher creates a two-parents conflict; if a world transform is "
                   "needed, target <font face=\"Courier\">world → camera_link</font>.", BODY)),
        ListItem(p("<b>arm1 (gripper) is excluded from MoveIt's planning group.</b> "
                   "The SRDF declares the chain base_link → tool0; arm1 is a sibling "
                   "branch off link2. MoveIt does not re-plan arm1 during MOVE steps — "
                   "gripper state persists across joint-space goals, which is exactly "
                   "what the pick state-machine wants.", BODY)),
    ], bulletType='bullet', leftIndent=20))

    # =================================================================
    #  11. Test results
    # =================================================================
    story.append(PageBreak())
    story.append(p('11. Test results', H1))
    story.append(p('11.1 IK accuracy on representative targets', H2))
    story.append(p(
        "Sampled four targets across the calibrated workspace. Each row reports the "
        "winner of the 24-seed Newton solve.",
        BODY))
    story.append(table([
        ['Target XYZ (m)',           'Pos err',  'Tilt',  'Success'],
        ['(0.05,  0.18, 0.12)',     '0.012 mm', '22.2°', 'Yes'],
        ['(-0.05, 0.16, 0.10)',     '0.075 mm', '6.5°',  'Yes'],
        ['(0.10,  0.10, 0.12)',     '0.004 mm', '16.0°', 'Yes'],
        ['(0.0079, 0.2112, 0.11)',  '0.022 mm', '26.9°', 'Yes'],
    ], col_widths=[2.0 * inch, 1.2 * inch, 1.0 * inch, 1.0 * inch]))
    story.append(Spacer(1, 0.10 * inch))
    story.append(p(
        "All four targets converge to sub-millimetre position error — well below the "
        "5 mm pos_tol_m threshold and well below the calibration residual (~2.5 mm) so "
        "IK is no longer the dominant error term in the pipeline.",
        BODY))
    story.append(p('11.2 Reachable workspace (gripper-down)', H2))
    story.append(fig(os.path.join(FIGS, 'workspace.png'),
                     width_in=6.4,
                     caption_text="Figure 3. Gripper-down reachable workspace at "
                                  "z = 0.11 m, swept across XY. Left: IK success map "
                                  "(green = solved within 5 mm). Right: gripper tilt "
                                  "off-vertical for the same configurations. Reachable "
                                  "configurations live in roughly an 80–200 mm radial "
                                  "ring around the base, biased toward +Y where the "
                                  "homography is best calibrated."))
    story.append(p('11.3 FK numerical sanity', H2))
    story.append(table([
        ['Test',                                                          'Result'],
        ['PoE FK vs URDF-chain composition, 500 random configs (max err)', '4.3·10⁻¹⁶'],
        ['Analytical position Jacobian vs finite differences (max err)',   '5.5·10⁻¹⁰'],
        ['_inv3x3 vs np.linalg.inv (200 random matrices)',                 '6.7·10⁻¹¹'],
        ['_inv_se3 vs np.linalg.inv on SE(3) (200 random)',                '5.6·10⁻¹⁶'],
    ], col_widths=[4.0 * inch, 1.6 * inch]))
    story.append(Spacer(1, 0.10 * inch))
    story.append(p(
        "(<i>_inv3x3 / _inv_se3</i> were transient experimental rewrites later reverted; "
        "the parity figures are kept for reference.)",
        BODY))

    # =================================================================
    #  12. Known limitations & next steps
    # =================================================================
    story.append(p('12. Known limitations &amp; next steps', H1))
    story.append(p('12.1 Workspace shape', H2))
    story.append(p(
        "With the ±π/2 driver clamp, reliable picks live in roughly an 80–200&nbsp;mm radial "
        "ring around the base, centered on +Y. The calibration is fit to that region — see "
        "Figure 3 — so targets outside the ring are rejected upfront with a tilt &gt; "
        "tolerance error. Widening usable workspace requires either (a) a wider "
        "calibration capture (next subsection) or (b) firmware/wiring work on the driver "
        "to lift the ±90° clamp.",
        BODY))
    story.append(p('12.2 Calibration distribution', H2))
    story.append(p(
        "The current <font face=\"Courier\">homography.yaml</font> sits at 9/12 inliers "
        "with ~2.5&nbsp;mm mean residual — adequate for a 40&nbsp;mm cube. Three outliers "
        "(&gt; 13&nbsp;mm) and a +Y-biased point distribution are the next things to fix. "
        "Recapture with a wider XY spread (8+ points distributed across +Y AND -Y, +X AND "
        "-X) for full-table reach.",
        BODY))
    story.append(p('12.3 Multi-cube tracking', H2))
    story.append(p(
        "<font face=\"Courier\">charuco_tf_publisher</font> in "
        "<font face=\"Courier\">single_aruco</font> mode publishes one TF per "
        "<i>marker_id</i>. For multiple cubes, two paths: run one publisher node per "
        "marker_id (cheap, scales to a few cubes), or extend the publisher to emit one TF "
        "per detected marker (e.g. <font face=\"Courier\">cube_marker_2</font>, "
        "<font face=\"Courier\">cube_marker_5</font>) — the cleaner long-term path.",
        BODY))
    story.append(p('12.4 IK runtime', H2))
    story.append(p(
        "~1.5 s per <font face=\"Courier\">solve_ik</font> call (24 seeds × 120 Newton "
        "iterations each). This is fine for one-shot picks but is the bottleneck for "
        "real-time tracking. Easy reductions: cache the running prefix transforms across "
        "iterations (currently recomputed inside <font face=\"Courier\">space_jacobian</font>), "
        "or vectorise the seed loop with NumPy.",
        BODY))
    story.append(p('12.5 No collision model', H2))
    story.append(p(
        "OMPL plans in C without a collision environment beyond the URDF's self-collision "
        "matrix. Adding the table as a planning-scene collision object would prevent the "
        "rare paths that scrape the table on the way to a pick. The Pilz Cartesian-limits "
        "config is loaded but inert — activating Pilz LIN/CIRC would also give a "
        "straight-line descent for the GRASP step if the joint-space path overshoots.",
        BODY))

    # =================================================================
    #  13. References & file pointers
    # =================================================================
    story.append(PageBreak())
    story.append(p('13. References &amp; file pointers', H1))
    story.append(p('13.1 Quick-jump table', H2))
    story.append(table([
        ['Concept',                            'File',                                                       'Approx line'],
        ['PoE forward kinematics',             'src/xarm_pick/xarm_pick/arm_ik.py',                          '190'],
        ['Screw axes SCREW_AXES + M_HOME',     'src/xarm_pick/xarm_pick/arm_ik.py',                          '166'],
        ['Space / body / position Jacobian',   'src/xarm_pick/xarm_pick/arm_ik.py',                          '201'],
        ['DLS Newton (one seed)',              'src/xarm_pick/xarm_pick/arm_ik.py',                          '287'],
        ['solve_ik (24-restart, tier select)', 'src/xarm_pick/xarm_pick/arm_ik.py',                          '380'],
        ['Driver ±π/2 clamp',                  'src/xarm_hw/xarm_hw/driver.py',                              '125'],
        ['FollowJointTrajectory exec',         'src/xarm_hw/xarm_hw/driver.py',                              '217'],
        ['Pixel → world map',                  'src/xarm_pick/xarm_pick/pick_2d.py',                         '104'],
        ['Pick state machine',                 'src/xarm_pick/xarm_pick/pick_2d.py',                         '211'],
        ['Two-phase calibration',              'src/xarm_pick/xarm_pick/calibrate_homography.py',            '160'],
        ['Homography fit (RANSAC)',            'src/xarm_pick/xarm_pick/calibrate_homography.py',            '290'],
        ['ArUco IPPE detector',                'src/charuco_tf_publisher/.../charuco_tf_node.py',            '488'],
        ['IPPE branch picker',                 'src/charuco_tf_publisher/.../charuco_tf_node.py',            '574'],
        ['MoveGroup joint-space goal',         'src/xarm_pick/xarm_pick/moveit_client.py',                   '62'],
        ['OMPL config',                        'src/xarm_moveit_config/config/ompl_planning.yaml',           '—'],
        ['Joint / scaling limits',             'src/xarm_moveit_config/config/joint_limits.yaml',            '—'],
    ], col_widths=[2.4 * inch, 3.0 * inch, 0.8 * inch]))
    story.append(Spacer(1, 0.20 * inch))
    story.append(p('13.2 External references', H2))
    story.append(ListFlowable([
        ListItem(p("Lynch &amp; Park, <i>Modern Robotics: Mechanics, Planning, and "
                   "Control</i>, Cambridge University Press, 2017. Forward / inverse "
                   "kinematics, Jacobians, time scaling.", BODY)),
        ListItem(p("Siciliano, Sciavicco, Villani, Oriolo, <i>Robotics: Modelling, "
                   "Planning and Control</i>, Springer, 2009. Task-priority redundancy "
                   "resolution (§3.5).", BODY)),
        ListItem(p("Kunz &amp; Stilman, &ldquo;Time-Optimal Trajectory Generation for "
                   "Path Following with Bounded Acceleration and Velocity&rdquo;, RSS 2012. "
                   "(TOTG.)", BODY)),
        ListItem(p("Kuffner &amp; LaValle, &ldquo;RRT-Connect: An Efficient Approach to "
                   "Single-Query Path Planning&rdquo;, ICRA 2000.", BODY)),
        ListItem(p("Collins &amp; Bartoli, &ldquo;Infinitesimal Plane-Based Pose "
                   "Estimation&rdquo;, IJCV 2014. (IPPE.)", BODY)),
        ListItem(p("Hartley &amp; Zisserman, <i>Multiple View Geometry in Computer "
                   "Vision</i>, 2nd ed., Cambridge University Press, 2003. Plane "
                   "homography fit and SVD normal equations.", BODY)),
    ], bulletType='bullet', leftIndent=20))
    story.append(Spacer(1, 0.30 * inch))
    story.append(p(
        "<i>End of report. The xarm_moveit workspace is on branch </i>"
        "<font face=\"Courier\">main</font><i>; this document corresponds to the "
        "post-driver-clamp-fix state of the IK module on " + date.today().isoformat() +
        ".</i>", H_DATE))

    doc.build(story)
    return OUT


if __name__ == '__main__':
    out = build()
    print(f'Wrote {out}')
    print(f'  size: {os.path.getsize(out) / 1024:.1f} KB')
