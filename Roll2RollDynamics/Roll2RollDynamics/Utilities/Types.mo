within Roll2RollDynamics.Utilities;
package Types
  "Type definitions and parameter records for roll-to-roll components"
  extends Modelica.Icons.TypesPackage;

  constant Modelica.Units.SI.Length lengthTol = 1e-9
    "Smallest length used in a denominator, keeping spans and transformers regular";

  constant Modelica.Units.SI.Force forceTol = 1e-9
    "Smallest force used in a denominator, keeping a span angle inferred from tension regular";

  constant Modelica.Units.SI.Velocity velocityTol = 1e-9
    "Smallest velocity used in a denominator, keeping a slip direction regular when the web is at rest";

  constant Modelica.Units.SI.Inertia inertiaTol = 1e-6
    "Smallest moment of inertia used, keeping a variable inertia regular when a roll carries almost nothing";

  type WebFace = enumeration(
      Inner "Material face wound toward the roll core",
      Outer "Opposite material face")
    "Identity of the web face a contact acts on"
    annotation(Documentation(info = "<html>
<p>The web has two distinct material faces, usually with different friction
characteristics. <code>Inner</code> names the face wound toward the core of the
parent roll and <code>Outer</code> its opposite. The labels travel with the
material, so they do not change when the line runs in reverse. Each contact
selects the face it touches; a nip pressing on a wrapped roller necessarily
touches the other one.</p>
</html>"));

  record WebParameters
    "Properties of the web itself, shared by the spans and the rolls it touches"
    extends Modelica.Icons.Record;
    parameter Modelica.Units.SI.Length thickness = 0.005 "Web thickness";
    parameter Modelica.Units.SI.Length width = 0.6 "Web width (cross-machine)";
    parameter Modelica.Units.SI.Density density = 900 "Web material density";
    parameter Real EA(unit = "N") = 8000 "Axial stiffness E*A";
    annotation(
      Documentation(
        info = "<html>
<p>Properties of the web material. One web runs through the whole line, so these
are shared. Density and section refer to unstretched material and belong to
the line rather than to any one component. <code>Belt</code> uses
every field. <code>Roller</code> takes <code>thickness</code> for the belt
centreline radius, and <code>width</code> and <code>density</code> for the
wrapped web it draws and for the mass in the contact patch.
<code>Winder</code> takes <code>thickness</code>, which sets how fast the wound
radius changes, and <code>width</code> for the wound section.</p>
<h4>Notes</h4>
<p><code>width</code> is the width of the web, not of the roll that carries it.
Roll face width is a property of the roll and is given separately.</p>
</html>"
      )
    );
  end WebParameters;

  record TractionFrictionParameters
    "Triple-S web-to-roll traction parameters"
    extends Modelica.Icons.Record;
    parameter Real muAdhesion(unit = "1", min = 0) = 0.35
      "Peak friction coefficient in the adhesion region";
    parameter Real muSliding(unit = "1", min = 0) = 0.22
      "Friction coefficient after the sliding transition";
    parameter Modelica.Units.SI.Velocity vAdhesion(min = Modelica.Constants.small) = 1e-3
      "Relative velocity at the adhesion peak";
    parameter Modelica.Units.SI.Velocity vSlide(min = Modelica.Constants.small) = 3e-3
      "Relative velocity at saturated sliding friction";
    parameter Real viscousSlope(min = 0, unit = "s/m") = 0
      "Linear velocity-dependent friction coefficient per unit slip speed";
    annotation(Documentation(info = "<html>
<p>Parameters of the analytic Triple-S friction curve used between the web and
a roll surface. The curve rises to <code>muAdhesion</code> at
<code>vAdhesion</code> and settles to <code>muSliding</code> beyond
<code>vSlide</code>, so it reproduces the drop from static to sliding friction
without an event. The defaults set <code>vSlide = 3*vAdhesion</code>, matching
the velocity scale on which a classical Stribeck exponential has effectively
settled to sliding friction.</p>
<p><code>viscousSlope</code> is the linear term
<a href=\"modelica://Roll2RollDynamics.Functions.tripleS\">tripleS</a> rises on once
past sliding, and comes back inside the same coefficient rather than being
added to it by whatever uses the curve. Its default is
zero because the slope depends on the web, coating, roll surface, and contact
conditions and therefore needs measurement or calibration.</p>
<h4>Limitations</h4>
<p>The regularization is continuous, so it has no exactly locked state: a
wrapped web always creeps a little. <code>vAdhesion</code> sets how much. A
nonzero viscous slope makes the total friction grow without a high-speed
saturation limit.</p>
</html>"));
  end TractionFrictionParameters;
  annotation(
    Documentation(
      info = "<html>
<p>Parameter records and shared constants for the Roll to Roll Dynamics library. Values
that describe one physical thing are grouped into a record so that a line
states them once and passes them as a unit, rather than wiring the fields one
by one.</p>
</html>"
    )
  );
end Types;
