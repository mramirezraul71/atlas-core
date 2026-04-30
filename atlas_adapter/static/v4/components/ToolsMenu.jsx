/**
 * Reference component for teams that consume ATLAS UI in React.
 * Runtime dashboard v4 currently mounts `modules/tools_menu.js`.
 */
import React from "react";

export default function ToolsMenu() {
  return (
    <section>
      <h2>Tools Menu</h2>
      <p>
        Esta referencia existe para parity con flujos React. La implementacion
        activa en ATLAS v4 usa el modulo vanilla:
        <code>atlas_adapter/static/v4/modules/tools_menu.js</code>.
      </p>
    </section>
  );
}
