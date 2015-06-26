^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package katana_arm_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* gazebo urdfs: remove self_collide tags
  Due to fixed joint reduction, there is only one link in the resulting
  SDF anyway, so there is nothing to collide with. Apart from that, this
  never made any sense for static models anyway.
* self_collide false for tall table
  The links of the table are statically connected.
  There's no need for collision checking here.
  gazebo5 raises a warning because the self_collide statements
  were inconsistent (not all specified) before.
* Contributors: Martin Günther, Michael Görner

1.0.1 (2015-03-17)
------------------

1.0.0 (2015-03-16)
------------------
* Initial release to Debian packages
* Contributors: Martin Günther, Henning Deeken, Jochen Sprickerhof, Michael Görner, André Potenza, Karl Glatz
