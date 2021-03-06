# ----------------------------------------------------
# This file is generated by the Qt Visual Studio Tools.
# ------------------------------------------------------

TEMPLATE = app
TARGET = CloudViewer
DESTDIR = ../../Debug
QT += core opengl gui widgets multimediawidgets 3dcore 3dquick openglextensions
CONFIG += debug
DEFINES += _UNICODE WIN64 QT_3DCORE_LIB QT_3DANIMATION_LIB QT_3DEXTRAS_LIB QT_3DINPUT_LIB QT_3DLOGIC_LIB QT_3DRENDER_LIB QT_3DQUICK_LIB QT_3DQUICKANIMATION_LIB QT_3DQUICKEXTRAS_LIB QT_3DQUICKINPUT_LIB QT_3DQUICKRENDER_LIB QT_3DQUICKSCENE2D_LIB QT_MULTIMEDIAWIDGETS_LIB QT_OPENGL_LIB QT_OPENGLEXTENSIONS_LIB QT_WIDGETS_LIB _SCL_SECURE_NO_WARNINGS _CRT_SECURE_NO_WARNINGS PCL_NO_PRECOMPILE
INCLUDEPATH += ./GeneratedFiles \
    . \
    ./GeneratedFiles/$(ConfigurationName)
LIBS += -lopengl32 \
    -lglu32 \
    -lpcl_common_debug \
    -lpcl_features_debug \
    -lpcl_filters_debug \
    -lpcl_io_debug \
    -lpcl_io_ply_debug \
    -lpcl_kdtree_debug \
    -lpcl_keypoints_debug \
    -lpcl_ml_debug \
    -lpcl_octree_debug \
    -lpcl_outofcore_debug \
    -lpcl_people_debug \
    -lpcl_recognition_debug \
    -lpcl_registration_debug \
    -lpcl_sample_consensus_debug \
    -lpcl_search_debug \
    -lpcl_segmentation_debug \
    -lpcl_stereo_debug \
    -lpcl_surface_debug \
    -lpcl_tracking_debug \
    -lpcl_visualization_debug \
    -lflann_cpp_s-gd \
    -lflann_cpp-gd \
    -lflann_s-gd \
    -lflann-gd \
    -llibboost_atomic-vc141-mt-gd-1_64 \
    -llibboost_bzip2-vc141-mt-gd-1_64 \
    -llibboost_chrono-vc141-mt-gd-1_64 \
    -llibboost_container-vc141-mt-gd-1_64 \
    -llibboost_context-vc141-mt-gd-1_64 \
    -llibboost_coroutine-vc141-mt-gd-1_64 \
    -llibboost_date_time-vc141-mt-gd-1_64 \
    -llibboost_exception-vc141-mt-gd-1_64 \
    -llibboost_fiber-vc141-mt-gd-1_64 \
    -llibboost_filesystem-vc141-mt-gd-1_64 \
    -llibboost_graph_parallel-vc141-mt-gd-1_64 \
    -llibboost_graph-vc141-mt-gd-1_64 \
    -llibboost_iostreams-vc141-mt-gd-1_64 \
    -llibboost_locale-vc141-mt-gd-1_64 \
    -llibboost_log_setup-vc141-mt-gd-1_64 \
    -llibboost_log-vc141-mt-gd-1_64 \
    -llibboost_math_c99f-vc141-mt-gd-1_64 \
    -llibboost_math_c99l-vc141-mt-gd-1_64 \
    -llibboost_math_c99-vc141-mt-gd-1_64 \
    -llibboost_math_tr1f-vc141-mt-gd-1_64 \
    -llibboost_math_tr1l-vc141-mt-gd-1_64 \
    -llibboost_math_tr1-vc141-mt-gd-1_64 \
    -llibboost_mpi-vc141-mt-gd-1_64 \
    -llibboost_numpy3-vc141-mt-gd-1_64 \
    -llibboost_numpy-vc141-mt-gd-1_64 \
    -llibboost_prg_exec_monitor-vc141-mt-gd-1_64 \
    -llibboost_program_options-vc141-mt-gd-1_64 \
    -llibboost_python3-vc141-mt-gd-1_64 \
    -llibboost_python-vc141-mt-gd-1_64 \
    -llibboost_random-vc141-mt-gd-1_64 \
    -llibboost_regex-vc141-mt-gd-1_64 \
    -llibboost_serialization-vc141-mt-gd-1_64 \
    -llibboost_signals-vc141-mt-gd-1_64 \
    -llibboost_system-vc141-mt-gd-1_64 \
    -llibboost_test_exec_monitor-vc141-mt-gd-1_64 \
    -llibboost_thread-vc141-mt-gd-1_64 \
    -llibboost_timer-vc141-mt-gd-1_64 \
    -llibboost_type_erasure-vc141-mt-gd-1_64 \
    -llibboost_unit_test_framework-vc141-mt-gd-1_64 \
    -llibboost_wave-vc141-mt-gd-1_64 \
    -llibboost_wserialization-vc141-mt-gd-1_64 \
    -llibboost_zlib-vc141-mt-gd-1_64 \
    -lqhull_d \
    -lqhull_p_d \
    -lqhull_r_d \
    -lqhullcpp_d \
    -lqhullstatic_d \
    -lqhullstatic_r_d \
    -lOpenNI2 \
    -lvtkalglib-8.0 \
    -lvtkChartsCore-8.0 \
    -lvtkCommonColor-8.0 \
    -lvtkCommonComputationalGeometry-8.0 \
    -lvtkCommonCore-8.0 \
    -lvtkCommonDataModel-8.0 \
    -lvtkCommonExecutionModel-8.0 \
    -lvtkCommonMath-8.0 \
    -lvtkCommonMisc-8.0 \
    -lvtkCommonSystem-8.0 \
    -lvtkCommonTransforms-8.0 \
    -lvtkDICOMParser-8.0 \
    -lvtkDomainsChemistry-8.0 \
    -lvtkDomainsChemistryOpenGL2-8.0 \
    -lvtkexoIIc-8.0 \
    -lvtkexpat-8.0 \
    -lvtkFiltersAMR-8.0 \
    -lvtkFiltersCore-8.0 \
    -lvtkFiltersExtraction-8.0 \
    -lvtkFiltersFlowPaths-8.0 \
    -lvtkFiltersGeneral-8.0 \
    -lvtkFiltersGeneric-8.0 \
    -lvtkFiltersGeometry-8.0 \
    -lvtkFiltersHybrid-8.0 \
    -lvtkFiltersHyperTree-8.0 \
    -lvtkFiltersImaging-8.0 \
    -lvtkFiltersModeling-8.0 \
    -lvtkFiltersParallel-8.0 \
    -lvtkFiltersParallelImaging-8.0 \
    -lvtkFiltersPoints-8.0 \
    -lvtkFiltersProgrammable-8.0 \
    -lvtkFiltersSelection-8.0 \
    -lvtkFiltersSMP-8.0 \
    -lvtkFiltersSources-8.0 \
    -lvtkFiltersStatistics-8.0 \
    -lvtkFiltersTexture-8.0 \
    -lvtkFiltersTopology-8.0 \
    -lvtkFiltersVerdict-8.0 \
    -lvtkfreetype-8.0 \
    -lvtkGeovisCore-8.0 \
    -lvtkgl2ps-8.0 \
    -lvtkglew-8.0 \
    -lvtkGUISupportQt-8.0 \
    -lvtkGUISupportQtSQL-8.0 \
    -lvtkhdf5_hl-8.0 \
    -lvtkhdf5-8.0 \
    -lvtkImagingColor-8.0 \
    -lvtkImagingCore-8.0 \
    -lvtkImagingFourier-8.0 \
    -lvtkImagingGeneral-8.0 \
    -lvtkImagingHybrid-8.0 \
    -lvtkImagingMath-8.0 \
    -lvtkImagingMorphological-8.0 \
    -lvtkImagingSources-8.0 \
    -lvtkImagingStatistics-8.0 \
    -lvtkImagingStencil-8.0 \
    -lvtkInfovisCore-8.0 \
    -lvtkInfovisLayout-8.0 \
    -lvtkInteractionImage-8.0 \
    -lvtkInteractionStyle-8.0 \
    -lvtkInteractionWidgets-8.0 \
    -lvtkIOAMR-8.0 \
    -lvtkIOCore-8.0 \
    -lvtkIOEnSight-8.0 \
    -lvtkIOExodus-8.0 \
    -lvtkIOExport-8.0 \
    -lvtkIOExportOpenGL2-8.0 \
    -lvtkIOGeometry-8.0 \
    -lvtkIOImage-8.0 \
    -lvtkIOImport-8.0 \
    -lvtkIOInfovis-8.0 \
    -lvtkIOLegacy-8.0 \
    -lvtkIOLSDyna-8.0 \
    -lvtkIOMINC-8.0 \
    -lvtkIOMovie-8.0 \
    -lvtkIONetCDF-8.0 \
    -lvtkIOParallel-8.0 \
    -lvtkIOParallelXML-8.0 \
    -lvtkIOPLY-8.0 \
    -lvtkIOSQL-8.0 \
    -lvtkIOTecplotTable-8.0 \
    -lvtkIOVideo-8.0 \
    -lvtkIOXML-8.0 \
    -lvtkIOXMLParser-8.0 \
    -lvtkjpeg-8.0 \
    -lvtkjsoncpp-8.0 \
    -lvtklibharu-8.0 \
    -lvtklibxml2-8.0 \
    -lvtklz4-8.0 \
    -lvtkmetaio-8.0 \
    -lvtknetcdf_c++ \
    -lvtkNetCDF-8.0 \
    -lvtkoggtheora-8.0 \
    -lvtkParallelCore-8.0 \
    -lvtkpng-8.0 \
    -lvtkproj4-8.0 \
    -lvtkRenderingAnnotation-8.0 \
    -lvtkRenderingContext2D-8.0 \
    -lvtkRenderingContextOpenGL2-8.0 \
    -lvtkRenderingCore-8.0 \
    -lvtkRenderingFreeType-8.0 \
    -lvtkRenderingGL2PSOpenGL2-8.0 \
    -lvtkRenderingImage-8.0 \
    -lvtkRenderingLabel-8.0 \
    -lvtkRenderingLOD-8.0 \
    -lvtkRenderingOpenGL2-8.0 \
    -lvtkRenderingQt-8.0 \
    -lvtkRenderingVolume-8.0 \
    -lvtkRenderingVolumeOpenGL2-8.0 \
    -lvtksqlite-8.0 \
    -lvtksys-8.0 \
    -lvtkTestingGenericBridge-8.0 \
    -lvtkTestingIOSQL-8.0 \
    -lvtkTestingRendering-8.0 \
    -lvtktiff-8.0 \
    -lvtkverdict-8.0 \
    -lvtkViewsContext2D-8.0 \
    -lvtkViewsCore-8.0 \
    -lvtkViewsInfovis-8.0 \
    -lvtkViewsQt-8.0 \
    -lvtkzlib-8.0 \
    -lQVTKWidgetPlugin
DEPENDPATH += .
MOC_DIR += ./GeneratedFiles/$(ConfigurationName)
OBJECTS_DIR += debug
UI_DIR += ./GeneratedFiles
RCC_DIR += ./GeneratedFiles
include(CloudViewer.pri)

TRANSLATIONS += ./TSResource/lang_en.ts \
    ./TSResource/lang_zh_CN.ts
