# Axe Maillage surfacique 3D

##    Biblio 

point d'entrée pour initier la biblio :

State of the Art in Surface Reconstruction from Point Clouds (2014_BTSALSS_star_author)

le pptx Organisation_Biblio_Mesh_3D est là pour vous aiguiller (non exhaustive)

Objectifs : 
 * comprendre la problématique 
 * identifier les COTS sur lesquels on peut s'appuyer, notamment la pertinence de CGAL
 * caractériser les méthodes
 * étudier la robustesse aux défauts de points clouds
 * comprendre l'algorithmie sous jacente aux grandes familles

A priori la librairie [CGAL](https://doc.cgal.org/latest/Mesh_3/index.html) sera utilisée pour les implémentations (sauf proposition différente à l'issue de la bibliographie). Il conviendra donc de lister les méthodes déjà présentes dans celle-ci, et proposer une ou plusier méthode à tester, voire à implémentation.

Les articles du dossier Biblio sont présents pour initier celle-ci, mais ne doivent pas constituer vos seules sources de recherche. 

##    Implémentation

La librairie privilégiée sera [CGAL](https://doc.cgal.org/latest/Mesh_3/index.html)

##    Tests sur des cas simples bien définis type nuages de points lidar
##    Tests sur des cas dégradés (troués, avec erreurs...) : les nuages de points S2P
##   Projection du mesh en 2.5D, et comparaison avec la projection du nuage de points de S2P
