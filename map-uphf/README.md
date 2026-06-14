# Piste UPHF (Gyrovia) pour CARLA

Modelisation de la piste d'essai UPHF/Gyrovia (Valenciennes) pour CARLA 0.9.16.

## Fichiers
- `uphf_piste.fbx` : mesh de la piste (export RoadRunner)
- `uphf_piste_roadrunner.xodr` : reseau OpenDRIVE source (RoadRunner) —
  topologie complete, 183 segments, 2 impasses
- `uphf_piste_aligned.xodr` : xodr final, translate de (-4, +17) m pour
  s'aligner sur le mesh recentre par Import.py (alignement valide a 100%)
- `uphf_piste.geojson` : geometrie vectorielle (Lambert-93 EPSG:2154)
- `uphf_piste.rrdata.xml` : metadonnees RoadRunner

## Pipeline de creation
1. Donnees source : OSM + releve LiDAR Gyrovia (.las, non versionne)
2. Modelisation RoadRunner -> export FBX + xodr
3. Import CARLA via Util/BuildTools/Import.py (genere le package cooke)
4. Alignement xodr/mesh : Import.py recentre le mesh ; le xodr est translate
   de (-4, +17) m pour correspondre (voir uphf_piste_aligned.xodr)

## Integration CARLA
Package cooke non versionne (volumineux). Utiliser le Docker autonome :
voir ../docker/README_tuteur.md.

## Points cles (lecons apprises)
- Le cache TM .bin d'un build UE4 custom fait planter le TM runtime Docker :
  le supprimer du package (reconstruit depuis le xodr au 1er chargement).
- La collision PhysX doit etre cookee via Import.py (l'import manuel FBX ne
  survit pas au cooking binaire).
- Map par defaut via DefaultEngine.ini pour demarrage direct.
