# Octree : définition concise

Un octree est une hiérarchie d’emboîtement d’AABB alignées sur les axes, où chaque nœud représente un volume cubique (ou parallélépipédique) de l’espace 3D et peut se subdiviser en huit enfants couvrant chacun un octant. L’objectif est de ranger des objets selon leur position pour que les requêtes spatiales (proximité, visibilité, collisions) ne parcourent que les zones pertinentes.

Principes clés :
- **Subdivision adaptative** : tant qu’un nœud contient plus d’objets que toléré ou que sa profondeur autorise, il se scinde en huit sous-volumes. Les objets qui tiennent entièrement dans un enfant y sont déplacés, sinon ils restent au nœud courant.
- **Stockage local** : un nœud mémorise les objets qui le traversent et propulse dans ses enfants uniquement ceux qui sont entièrement contenus, évitant la duplication et conservant l’accès aux volumes chevauchants.
- **Parcours ciblé** : une requête parcourt seulement les nœuds dont l’AABB intersecte le volume recherché; on réduit ainsi le nombre de tests fins nécessaires.

Intérêt en détection de collisions : l’octree sert de phase élargie. On insère les volumes englobants (p. ex. sphères ou AABB) des objets en mouvement, puis on ne teste en phase restreinte que les couples d’objets trouvés dans les mêmes nœuds traversés par la requête. On filtre ainsi la combinatoire des collisions tout en laissant la résolution détaillée (contacts, impulsions) à la suite du pipeline.
