# Image Docker carla-uphf (8.5 Go)

L'image complete n'est PAS versionnee dans ce repo (trop volumineuse pour Git).

## Reconstruire l'image depuis ce dossier
Necessite le package map `uphf_gyrovia.tar.gz` et `python310-standalone.tar.gz`
(voir le responsable du projet pour les obtenir), puis :

    docker build --network=host -t carla-uphf:0.9.16 .

## Ou recuperer l'image pre-construite
[A COMPLETER : lien de telechargement / emplacement reseau du labo]
