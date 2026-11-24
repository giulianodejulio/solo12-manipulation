# Asset provenance

`assets/robot.xml` is the frozen initial assembly for the successful demonstration,
with relative mesh paths. It combines the following sources; it is not an original
Solo12 or ROBOTIS model authored by this project.

- **Solo12 MJCF and visual/collision meshes:**
  https://github.com/rkourdis/solo12_mujoco/tree/846cd588f461050068aaea03511915a8ddd8f06a
  The upstream README describes derivation from ODRI `robot_properties_solo` and the
  Open Dynamic Robot Initiative hardware project. The MuJoCo conversion repository
  does not contain an explicit license file at this revision. No additional license
  is claimed here for its conversion work. Meshes are downloaded directly from the
  pinned upstream, not redistributed in this Git repository.
- **OpenMANIPULATOR-X meshes and original inertials:**
  https://github.com/ROBOTIS-GIT/open_manipulator/tree/ee9a720bcc45acda9c0df8fe689b1182844307db
  Apache-2.0; upstream license reproduced in `licenses/ROBOTIS-Apache-2.0.txt`.
- **Underlying ODRI hardware reference:**
  https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware/tree/66af1522b4fba0ec4a1d7790e66f5e4652208d30
  BSD-3-Clause; reference notice in `licenses/ODRI-BSD-3-Clause.txt`. This notice
  does not imply that every upstream transformation has that same license.
- **Four support meshes in `assets/support/`:** project-generated front saddle
  at 37 mm and front crossmember. These are the final conceptual variant used by
  this demo, not manufacturing-ready parts. No earlier design revisions included.

`assets/manifest.json` lists exact upstream paths, revisions and SHA-256 hashes for
all required downloaded meshes. The loader never needs another local workspace.
