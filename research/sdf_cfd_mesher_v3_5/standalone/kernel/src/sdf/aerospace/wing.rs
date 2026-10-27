// Wing lofting with geometric parameters

use super::airfoil::{
    Airfoil, AirfoilExportOptions, AirfoilFeatureMetadata, get_naca_airfoil,
    get_naca_airfoil_export_safe,
};
use super::section::Section2D;
use crate::sdf::Sdf;
use crate::sdf::cfd_query::QueryValueStatus;
use crate::sdf::conditioning::{SdfBounds, SdfNodeMetadata};
use crate::sdf::procedural_features::{
    CapRole, FeatureEndpointRole, PROCEDURAL_FEATURE_SCHEMA_VERSION, ProceduralFeatureCapabilities,
    ProceduralFeatureDefinition, ProceduralFeatureDiagnostics, ProceduralFeatureInventory,
    ProceduralFeatureJunction, ProceduralFeatureMatch, ProceduralFeatureQueryOptions,
    ProceduralFeatureQueryResult, ProceduralFeatureType, SurfaceSectorDefinition, SurfaceSectorId,
    WingSurfaceSide, vec3_array,
};
use glam::{Vec2, Vec3};
use std::sync::Arc;

/// Wing section at a specific span location
#[derive(Clone)]
pub struct WingSection {
    pub section: Arc<dyn Section2D>,
    pub position: Vec3, // Leading edge position
    pub twist: f32,     // Twist angle in radians
}

/// Lofted wing created from cross-section slices
pub struct LoftedWing {
    pub sections: Vec<WingSection>,
}

impl LoftedWing {
    pub fn new(sections: Vec<WingSection>) -> Self {
        Self { sections }
    }

    pub fn feature_metadata(&self) -> Option<AirfoilFeatureMetadata> {
        self.sections
            .iter()
            .filter_map(|section| {
                section
                    .section
                    .as_any()
                    .downcast_ref::<Airfoil>()
                    .and_then(|airfoil| airfoil.feature_metadata)
            })
            .reduce(|acc, item| AirfoilFeatureMetadata {
                trailing_edge_thickness_mm: acc
                    .trailing_edge_thickness_mm
                    .min(item.trailing_edge_thickness_mm),
                leading_edge_radius_mm: acc.leading_edge_radius_mm.min(item.leading_edge_radius_mm),
                min_feature_size_mm: acc.min_feature_size_mm.min(item.min_feature_size_mm),
                export_safe: acc.export_safe || item.export_safe,
            })
    }
}

impl Sdf for LoftedWing {
    fn distance(&self, point: Vec3) -> f32 {
        crate::sdf::sdf_profile_node_visit();
        if !point.is_finite() {
            return f32::NAN;
        }
        if self.sections.len() < 2 {
            return point.length();
        }

        // Span is along Y; cross-section lives in the XZ plane.
        let selection_timer=super::naca_spline::profile::start();
        let span_pos = point.y;

        let y_min = self
            .sections
            .iter()
            .map(|s| s.position.y)
            .fold(f32::MAX, f32::min);
        let y_max = self
            .sections
            .iter()
            .map(|s| s.position.y)
            .fold(f32::MIN, f32::max);

        let d_span = (y_min - span_pos).max(span_pos - y_max);
        let y_clamped = span_pos.clamp(y_min, y_max);

        // Find segment pair that brackets y_clamped
        let n = self.sections.len();
        let mut seg_idx = n - 2;
        let mut t = 1.0f32;
        for i in 0..n - 1 {
            let ya = self.sections[i].position.y;
            let yb = self.sections[i + 1].position.y;
            let span = yb - ya;
            if span.abs() < 1e-6 {
                continue;
            }
            let t_raw = (y_clamped - ya) / span;
            if t_raw >= 0.0 && t_raw <= 1.0 {
                seg_idx = i;
                t = t_raw;
                break;
            }
        }

        let sec_a = &self.sections[seg_idx];
        let sec_b = &self.sections[seg_idx + 1];

        // Interpolate leading-edge position and twist at y_clamped
        let lerp_x = sec_a.position.x + t * (sec_b.position.x - sec_a.position.x);
        let lerp_z = sec_a.position.z + t * (sec_b.position.z - sec_a.position.z);
        let lerp_twist = sec_a.twist + t * (sec_b.twist - sec_a.twist);

        // Cross-section is in XZ plane; X = chord direction, Z = thickness direction
        let local = Vec2::new(point.x - lerp_x, point.z - lerp_z);

        // Twist rotates in XZ plane
        let (sin_t, cos_t) = (-lerp_twist).sin_cos();
        let rotated = Vec2::new(
            local.x * cos_t - local.y * sin_t,
            local.x * sin_t + local.y * cos_t,
        );

        // True lofted cross-section: blend sec_a and sec_b shapes at parameter t.
        super::naca_spline::profile::end("wing_selection_interpolation",selection_timer);
        // distance_lerped_2d is allocation-free — critical for the realtime hot path.
        let dist_2d = sec_a
            .section
            .distance_lerped_2d(&*sec_b.section, t, rotated);

        // Float max/min suppress NaN. A failed section query must not become
        // a finite cap-only distance and materialize an unresolved section.
        if !dist_2d.is_finite() || !d_span.is_finite() {
            return f32::NAN;
        }

        let outside = Vec2::new(dist_2d.max(0.0), d_span.max(0.0)).length();
        let inside = dist_2d.max(d_span).min(0.0);
        outside + inside
    }

    fn metadata(&self) -> SdfNodeMetadata {
        let mut points = Vec::new();
        let mut leading_edge_region_points = Vec::new();
        let mut trailing_edge_region_points = Vec::new();
        let mut fingerprint_values = Vec::new();
        let feature_metadata = self.feature_metadata();
        fingerprint_values.push(self.sections.len() as f32);
        for section in &self.sections {
            let Some(bounds) = section.section.bounds_2d() else {
                return SdfNodeMetadata::new("lofted_wing").with_approximate_bounds();
            };
            fingerprint_values.extend_from_slice(&[
                section.position.x,
                section.position.y,
                section.position.z,
                section.twist,
            ]);
            fingerprint_values.extend(section.section.metadata_fingerprint_parameters());
            let (sin_t, cos_t) = section.twist.sin_cos();
            for corner in bounds.corners() {
                let x = corner.x * cos_t - corner.y * sin_t;
                let z = corner.x * sin_t + corner.y * cos_t;
                points.push(Vec3::new(
                    section.position.x + x,
                    section.position.y,
                    section.position.z + z,
                ));
            }
            if let Some(feature_metadata) = feature_metadata {
                let margin = (feature_metadata.min_feature_size_mm * 4.0).max(1.0);
                let leading_bounds = super::section::SectionBounds2D::new(
                    Vec2::new(bounds.min.x - margin, -margin),
                    Vec2::new(bounds.min.x + margin, margin),
                );
                let trailing_bounds = super::section::SectionBounds2D::new(
                    Vec2::new(bounds.max.x - margin, -margin),
                    Vec2::new(bounds.max.x + margin, margin),
                );
                for (region_bounds, region_points) in [
                    (leading_bounds, &mut leading_edge_region_points),
                    (trailing_bounds, &mut trailing_edge_region_points),
                ] {
                    for corner in region_bounds.corners() {
                        let x = corner.x * cos_t - corner.y * sin_t;
                        let z = corner.x * sin_t + corner.y * cos_t;
                        region_points.push(Vec3::new(
                            section.position.x + x,
                            section.position.y,
                            section.position.z + z,
                        ));
                    }
                }
            }
        }

        match SdfBounds::from_points(&points) {
            Some(bounds) => {
                let mut metadata = SdfNodeMetadata::new("lofted_wing")
                    .with_support_bounds(bounds)
                    .with_approximate_bounds()
                    .with_f32_parameters(fingerprint_values);
                if let Some(feature_metadata) = self.feature_metadata() {
                    metadata = metadata.with_min_feature_size(feature_metadata.min_feature_size_mm);
                    if let Some(bounds) = SdfBounds::from_points(&leading_edge_region_points) {
                        metadata = metadata.with_feature_region(
                            "lofted_wing.leading_edge",
                            bounds,
                            feature_metadata.leading_edge_radius_mm * 2.0,
                        );
                    }
                    if let Some(bounds) = SdfBounds::from_points(&trailing_edge_region_points) {
                        metadata = metadata.with_feature_region(
                            "lofted_wing.trailing_edge",
                            bounds,
                            feature_metadata.trailing_edge_thickness_mm,
                        );
                    }
                }
                metadata
            }
            None => SdfNodeMetadata::new("lofted_wing").with_approximate_bounds(),
        }
    }

    fn procedural_feature_inventory(&self) -> Option<ProceduralFeatureInventory> {
        self.procedural_feature_inventory_impl()
    }

    fn evaluate_procedural_feature(
        &self,
        feature_id: &str,
        parameter: f64,
    ) -> Option<ProceduralFeatureQueryResult> {
        self.evaluate_feature_impl(feature_id, parameter)
    }

    fn query_procedural_feature(
        &self,
        point: Vec3,
        options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureQueryResult {
        self.query_feature_impl(point, options)
    }
}

const WING_COMPONENT_ID: &str = "lofted_wing.main";
const LE_ID: &str = "lofted_wing.main.leading_edge";
const TE_UPPER_ID: &str = "lofted_wing.main.trailing_edge.upper_cap_boundary";
const TE_LOWER_ID: &str = "lofted_wing.main.trailing_edge.lower_cap_boundary";
const TE_CAP_ID: &str = "lofted_wing.main.trailing_edge.cap_sector";
const TIP_POS_UPPER_ID: &str = "lofted_wing.main.tip.positive.upper_transition";
const TIP_POS_LOWER_ID: &str = "lofted_wing.main.tip.positive.lower_transition";
const TIP_NEG_UPPER_ID: &str = "lofted_wing.main.tip.negative.upper_transition";
const TIP_NEG_LOWER_ID: &str = "lofted_wing.main.tip.negative.lower_transition";
const TIP_POS_SECTOR_ID: &str = "lofted_wing.main.tip.positive.cap_sector";
const TIP_NEG_SECTOR_ID: &str = "lofted_wing.main.tip.negative.cap_sector";
const ROOT_SECTION_ID: &str = "lofted_wing.main.root.section";
const UPPER_SECTOR_ID: &str = "lofted_wing.main.surface.upper";
const LOWER_SECTOR_ID: &str = "lofted_wing.main.surface.lower";
const SKIN_ID: &str = "lofted_wing.main.surface.smooth";
const SMOOTH_CAP_CURVES: [&str;2] = ["lofted_wing.main.tip.negative.contour", "lofted_wing.main.tip.positive.contour"];
const TRAILING_CAP_CURVES: [&str;2] = ["lofted_wing.main.tip.negative.trailing", "lofted_wing.main.tip.positive.trailing"];

impl LoftedWing {
    fn smooth_sections(&self) -> bool {
        self.sections.len()>=2 && self.sections.iter().all(|s|s.section.as_any().downcast_ref::<Airfoil>().is_some_and(|a|a.spline().is_some()))
    }

    /// Constant-section extrusions have exactly the section's two boundary
    /// branches. A more general loft needs an explicit span-junction audit.
    fn smooth_extrusion(&self) -> bool {
        if !self.smooth_sections(){return false}
        let a=&self.sections[0]; let aa=a.section.as_any().downcast_ref::<Airfoil>().unwrap();
        self.sections.iter().all(|b| {
            let bb=b.section.as_any().downcast_ref::<Airfoil>().unwrap();
            b.position.x==a.position.x && b.position.z==a.position.z && b.twist==a.twist && bb.chord==aa.chord && bb.spline()==aa.spline()
        })
    }

    fn smooth_inventory(&self) -> ProceduralFeatureInventory {
        if !self.smooth_extrusion(){return ProceduralFeatureInventory::unsupported("lofted_wing","smooth section loft requires span-junction topology authority");}
        let mut features=Vec::new();let mut sectors=Vec::new();let mut junctions=Vec::new();
        let te=self.feature_metadata().map(|m|m.trailing_edge_thickness_mm as f64);
        for (id,kind) in [(TE_UPPER_ID,ProceduralFeatureType::TrailingEdgeUpperCapBoundary),(TE_LOWER_ID,ProceduralFeatureType::TrailingEdgeLowerCapBoundary)] {
            features.push(feature_definition(id,kind,vec![FeatureEndpointRole::NegativeTip,FeatureEndpointRole::PositiveTip],SMOOTH_CAP_CURVES.iter().chain(TRAILING_CAP_CURVES.iter()).copied().collect(),true,te));
        }
        for end in 0..SMOOTH_CAP_CURVES.len() {
            let curve=SMOOTH_CAP_CURVES[end];let trailing=TRAILING_CAP_CURVES[end];
            for id in [curve,trailing] {features.push(feature_definition(id,ProceduralFeatureType::TipPerimeter,vec![FeatureEndpointRole::TrailingUpperTip,FeatureEndpointRole::TrailingLowerTip],vec![TE_UPPER_ID,TE_LOWER_ID,if id==curve{trailing}else{curve}],true,te));}
            let cap=if end==0{TIP_NEG_SECTOR_ID}else{TIP_POS_SECTOR_ID};
            sectors.push(sector(cap,ProceduralFeatureType::TipCapSector,vec![curve,trailing]));
            for (role,span_edge,label) in [(FeatureEndpointRole::TrailingUpperTip,TE_UPPER_ID,"upper"),(FeatureEndpointRole::TrailingLowerTip,TE_LOWER_ID,"lower")] {
                junctions.push(junction(&format!("{}.{}",trailing,label),role,vec![curve,trailing,span_edge],Some(end as f64)));
            }
        }
        let mut skin=sector(SKIN_ID,ProceduralFeatureType::UpperSurfaceSector,vec![TE_UPPER_ID,TE_LOWER_ID,SMOOTH_CAP_CURVES[0],SMOOTH_CAP_CURVES[1]]);
        skin.notes.push("One C2 skin contains the upper branch, leading edge and lower branch; internal spline knots are not features.".into());sectors.push(skin);
        sectors.push(sector(TE_CAP_ID,ProceduralFeatureType::TrailingEdgeCapSector,vec![TE_UPPER_ID,TE_LOWER_ID,TRAILING_CAP_CURVES[0],TRAILING_CAP_CURVES[1]]));
        ProceduralFeatureInventory{schema_version:PROCEDURAL_FEATURE_SCHEMA_VERSION.into(),component_id:WING_COMPONENT_ID.into(),component_kind:"lofted_wing".into(),features,sectors,junctions,
            capabilities:ProceduralFeatureCapabilities{support_status:QueryValueStatus::Native,inventory:QueryValueStatus::Native,feature_evaluation:QueryValueStatus::Native,closest_feature:QueryValueStatus::Native,surface_sector:QueryValueStatus::Native,local_parameters:QueryValueStatus::Native,transform_propagation:QueryValueStatus::Native,limitations:vec!["Physical topology of a constant-section spline extrusion; root and leading-edge guides are not sharp features.".into()]}}
    }

    fn procedural_feature_inventory_impl(&self) -> Option<ProceduralFeatureInventory> {
        if self.smooth_sections(){return Some(self.smooth_inventory())}
        if self.sections.len() < 2 || !self.sections_are_airfoils() {
            return None;
        }
        let te = self
            .feature_metadata()
            .map(|m| m.trailing_edge_thickness_mm as f64);
        let mut features = Vec::new();
        features.push(feature_definition(
            LE_ID,
            ProceduralFeatureType::LeadingEdge,
            vec![
                FeatureEndpointRole::NegativeTip,
                FeatureEndpointRole::PositiveTip,
            ],
            vec![
                TIP_NEG_UPPER_ID,
                TIP_NEG_LOWER_ID,
                TIP_POS_UPPER_ID,
                TIP_POS_LOWER_ID,
            ],
            true,
            None,
        ));
        features.push(feature_definition(
            TE_UPPER_ID,
            ProceduralFeatureType::TrailingEdgeUpperCapBoundary,
            vec![
                FeatureEndpointRole::TrailingUpperTip,
                FeatureEndpointRole::TrailingUpperTip,
            ],
            vec![TE_CAP_ID, TIP_NEG_UPPER_ID, TIP_POS_UPPER_ID],
            true,
            te,
        ));
        features.push(feature_definition(
            TE_LOWER_ID,
            ProceduralFeatureType::TrailingEdgeLowerCapBoundary,
            vec![
                FeatureEndpointRole::TrailingLowerTip,
                FeatureEndpointRole::TrailingLowerTip,
            ],
            vec![TE_CAP_ID, TIP_NEG_LOWER_ID, TIP_POS_LOWER_ID],
            true,
            te,
        ));
        for (id, kind) in [
            (TIP_NEG_UPPER_ID, ProceduralFeatureType::TipUpperCapBoundary),
            (TIP_NEG_LOWER_ID, ProceduralFeatureType::TipLowerCapBoundary),
            (TIP_POS_UPPER_ID, ProceduralFeatureType::TipUpperCapBoundary),
            (TIP_POS_LOWER_ID, ProceduralFeatureType::TipLowerCapBoundary),
        ] {
            features.push(feature_definition(
                id,
                kind,
                vec![
                    FeatureEndpointRole::LeadingTip,
                    FeatureEndpointRole::TrailingUpperTip,
                ],
                vec![LE_ID, TE_UPPER_ID, TE_LOWER_ID],
                true,
                te,
            ));
        }
        features.push(feature_definition(
            ROOT_SECTION_ID,
            ProceduralFeatureType::RootSection,
            vec![FeatureEndpointRole::ClosedLoop],
            vec![LE_ID, TE_UPPER_ID, TE_LOWER_ID],
            false,
            te,
        ));

        let sectors = vec![
            sector(
                UPPER_SECTOR_ID,
                ProceduralFeatureType::UpperSurfaceSector,
                vec![LE_ID, TE_UPPER_ID],
            ),
            sector(
                LOWER_SECTOR_ID,
                ProceduralFeatureType::LowerSurfaceSector,
                vec![LE_ID, TE_LOWER_ID],
            ),
            sector(
                TE_CAP_ID,
                ProceduralFeatureType::TrailingEdgeCapSector,
                vec![TE_UPPER_ID, TE_LOWER_ID],
            ),
            sector(
                TIP_NEG_SECTOR_ID,
                ProceduralFeatureType::TipCapSector,
                vec![TIP_NEG_UPPER_ID, TIP_NEG_LOWER_ID],
            ),
            sector(
                TIP_POS_SECTOR_ID,
                ProceduralFeatureType::TipCapSector,
                vec![TIP_POS_UPPER_ID, TIP_POS_LOWER_ID],
            ),
        ];
        let junctions = vec![
            junction(
                "lofted_wing.main.junction.negative_tip.leading",
                FeatureEndpointRole::LeadingTip,
                vec![LE_ID, TIP_NEG_UPPER_ID, TIP_NEG_LOWER_ID],
                Some(0.0),
            ),
            junction(
                "lofted_wing.main.junction.positive_tip.leading",
                FeatureEndpointRole::LeadingTip,
                vec![LE_ID, TIP_POS_UPPER_ID, TIP_POS_LOWER_ID],
                Some(1.0),
            ),
            junction(
                "lofted_wing.main.junction.negative_tip.trailing_upper",
                FeatureEndpointRole::TrailingUpperTip,
                vec![TE_UPPER_ID, TIP_NEG_UPPER_ID],
                Some(0.0),
            ),
            junction(
                "lofted_wing.main.junction.negative_tip.trailing_lower",
                FeatureEndpointRole::TrailingLowerTip,
                vec![TE_LOWER_ID, TIP_NEG_LOWER_ID],
                Some(0.0),
            ),
            junction(
                "lofted_wing.main.junction.positive_tip.trailing_upper",
                FeatureEndpointRole::TrailingUpperTip,
                vec![TE_UPPER_ID, TIP_POS_UPPER_ID],
                Some(1.0),
            ),
            junction(
                "lofted_wing.main.junction.positive_tip.trailing_lower",
                FeatureEndpointRole::TrailingLowerTip,
                vec![TE_LOWER_ID, TIP_POS_LOWER_ID],
                Some(1.0),
            ),
            junction(
                "lofted_wing.main.root.section",
                FeatureEndpointRole::Root,
                vec![ROOT_SECTION_ID],
                Some(self.root_span_parameter()?),
            ),
        ];

        Some(ProceduralFeatureInventory {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            component_id: WING_COMPONENT_ID.to_string(),
            component_kind: "lofted_wing".to_string(),
            features,
            sectors,
            junctions,
            capabilities: ProceduralFeatureCapabilities {
                support_status: QueryValueStatus::Native,
                inventory: QueryValueStatus::Native,
                feature_evaluation: QueryValueStatus::Native,
                closest_feature: QueryValueStatus::Native,
                surface_sector: QueryValueStatus::Native,
                local_parameters: QueryValueStatus::Native,
                transform_propagation: QueryValueStatus::Native,
                limitations: vec![
                    "root section is a configured internal guide for the symmetric tested wing; no physical root cap surface exists".to_string(),
                    "tip transition curves are procedural cap-boundary parameter curves, not assertions of geometric sharpness".to_string(),
                ],
            },
        })
    }

    fn evaluate_feature_impl(
        &self,
        feature_id: &str,
        parameter: f64,
    ) -> Option<ProceduralFeatureQueryResult> {
        let parameter = parameter.clamp(0.0, 1.0);
        if self.smooth_sections() && (SMOOTH_CAP_CURVES.contains(&feature_id)||TRAILING_CAP_CURVES.contains(&feature_id)) {
            if !self.smooth_extrusion(){return None}
            let end=SMOOTH_CAP_CURVES.iter().position(|id|*id==feature_id).or_else(||TRAILING_CAP_CURVES.iter().position(|id|*id==feature_id))?;
            let frame=self.section_frame_at_span_parameter(end as f64)?;
            let a=self.sections[frame.seg].section.as_any().downcast_ref::<Airfoil>()?;
            let spline=a.spline()?;
            let (p,d)=if SMOOTH_CAP_CURVES.contains(&feature_id){
                let u=2.0*parameter-1.0;let (p,d)=spline.evaluate(u<=0.0,u.abs());(p,if u<=0.0{-d}else{d})
            } else {let u=spline.evaluate(true,1.0).0;let l=spline.evaluate(false,1.0).0;(u+(l-u)*parameter,l-u)};
            let point=frame.local_to_world((p*a.chord as f64).as_vec2());let tangent=(frame.local_to_world((d.normalize()*a.chord as f64).as_vec2())-frame.leading_edge).normalize();
            let mut result=ProceduralFeatureQueryResult::unsupported(point,"initializing exact spline feature");
            result.component_id=Some(WING_COMPONENT_ID.into());result.feature_id=Some(feature_id.into());result.feature_type=ProceduralFeatureType::TipPerimeter;result.support_status=QueryValueStatus::Native;
            result.closest_point=Some(vec3_array(point));result.distance_to_feature=Some(0.0);result.tangent=Some(vec3_array(tangent));result.span_parameter=Some(end as f64);result.airfoil_parameter=Some(parameter);
            result.endpoint_role=Some(if parameter==0.0{FeatureEndpointRole::TrailingUpperTip}else if parameter==1.0{FeatureEndpointRole::TrailingLowerTip}else{FeatureEndpointRole::Interior});
            result.connected_feature_ids=self.connected_features_for(feature_id);result.normal_sector=Some(SurfaceSectorId{stable_id:if end==0{TIP_NEG_SECTOR_ID.into()}else{TIP_POS_SECTOR_ID.into()},support_status:QueryValueStatus::Native});
            result.diagnostics.success=true;result.diagnostics.errors.clear();result.diagnostics.method="continuous spline authority".into();result.diagnostics.feature_to_sdf_residual=Some(self.distance(point) as f64);return Some(result);
        }
        let (point, tangent, feature_type, side, cap_role, endpoint_role, connected) =
            match feature_id {
                LE_ID => (
                    self.span_curve_point(parameter, AirfoilFeaturePoint::Leading)?,
                    self.span_curve_tangent(parameter, AirfoilFeaturePoint::Leading)?,
                    ProceduralFeatureType::LeadingEdge,
                    WingSurfaceSide::Unknown,
                    CapRole::None,
                    span_endpoint(parameter),
                    vec![
                        TIP_NEG_UPPER_ID,
                        TIP_NEG_LOWER_ID,
                        TIP_POS_UPPER_ID,
                        TIP_POS_LOWER_ID,
                    ],
                ),
                TE_UPPER_ID => (
                    self.span_curve_point(parameter, AirfoilFeaturePoint::TrailingUpper)?,
                    self.span_curve_tangent(parameter, AirfoilFeaturePoint::TrailingUpper)?,
                    ProceduralFeatureType::TrailingEdgeUpperCapBoundary,
                    WingSurfaceSide::Upper,
                    CapRole::TrailingEdgeCap,
                    span_endpoint(parameter),
                    vec![TE_CAP_ID, TIP_NEG_UPPER_ID, TIP_POS_UPPER_ID],
                ),
                TE_LOWER_ID => (
                    self.span_curve_point(parameter, AirfoilFeaturePoint::TrailingLower)?,
                    self.span_curve_tangent(parameter, AirfoilFeaturePoint::TrailingLower)?,
                    ProceduralFeatureType::TrailingEdgeLowerCapBoundary,
                    WingSurfaceSide::Lower,
                    CapRole::TrailingEdgeCap,
                    span_endpoint(parameter),
                    vec![TE_CAP_ID, TIP_NEG_LOWER_ID, TIP_POS_LOWER_ID],
                ),
                TIP_NEG_UPPER_ID => (
                    self.tip_curve_point(false, true, parameter)?,
                    self.tip_curve_tangent(false, true, parameter)?,
                    ProceduralFeatureType::TipUpperCapBoundary,
                    WingSurfaceSide::Upper,
                    CapRole::TipCapNegative,
                    tip_endpoint(parameter, true),
                    vec![LE_ID, TE_UPPER_ID, TIP_NEG_SECTOR_ID],
                ),
                TIP_NEG_LOWER_ID => (
                    self.tip_curve_point(false, false, parameter)?,
                    self.tip_curve_tangent(false, false, parameter)?,
                    ProceduralFeatureType::TipLowerCapBoundary,
                    WingSurfaceSide::Lower,
                    CapRole::TipCapNegative,
                    tip_endpoint(parameter, false),
                    vec![LE_ID, TE_LOWER_ID, TIP_NEG_SECTOR_ID],
                ),
                TIP_POS_UPPER_ID => (
                    self.tip_curve_point(true, true, parameter)?,
                    self.tip_curve_tangent(true, true, parameter)?,
                    ProceduralFeatureType::TipUpperCapBoundary,
                    WingSurfaceSide::Upper,
                    CapRole::TipCapPositive,
                    tip_endpoint(parameter, true),
                    vec![LE_ID, TE_UPPER_ID, TIP_POS_SECTOR_ID],
                ),
                TIP_POS_LOWER_ID => (
                    self.tip_curve_point(true, false, parameter)?,
                    self.tip_curve_tangent(true, false, parameter)?,
                    ProceduralFeatureType::TipLowerCapBoundary,
                    WingSurfaceSide::Lower,
                    CapRole::TipCapPositive,
                    tip_endpoint(parameter, false),
                    vec![LE_ID, TE_LOWER_ID, TIP_POS_SECTOR_ID],
                ),
                ROOT_SECTION_ID => (
                    self.root_section_point(parameter)?,
                    self.root_section_tangent(parameter)?,
                    ProceduralFeatureType::RootSection,
                    WingSurfaceSide::Unknown,
                    CapRole::None,
                    Some(FeatureEndpointRole::Root),
                    vec![LE_ID, TE_UPPER_ID, TE_LOWER_ID],
                ),
                _ => return None,
            };

        Some(ProceduralFeatureQueryResult {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            query_position: vec3_array(point),
            component_id: Some(WING_COMPONENT_ID.to_string()),
            feature_id: Some(feature_id.to_string()),
            feature_type,
            support_status: QueryValueStatus::Native,
            closest_point: Some(vec3_array(point)),
            distance_to_feature: Some(0.0),
            tangent: Some(vec3_array(tangent.normalize_or_zero())),
            normal_sector: self.sector_for_side(side, cap_role),
            span_parameter: if feature_id == ROOT_SECTION_ID {
                self.root_span_parameter()
            } else if feature_id.starts_with("lofted_wing.main.tip.") {
                Some(if feature_id.contains(".positive.") {
                    1.0
                } else {
                    0.0
                })
            } else {
                Some(parameter)
            },
            chord_parameter: self.chord_parameter_for_feature(feature_id, parameter),
            airfoil_parameter: if feature_id == ROOT_SECTION_ID || feature_id.contains(".tip.") {
                Some(parameter)
            } else {
                None
            },
            side: Some(side),
            cap_role: Some(cap_role),
            endpoint_role,
            connected_feature_ids: if self.smooth_sections(){self.connected_features_for(feature_id)}else{connected.into_iter().map(str::to_string).collect()},
            all_matches: Vec::new(),
            diagnostics: ProceduralFeatureDiagnostics {
                success: true,
                warnings: Vec::new(),
                errors: Vec::new(),
                ambiguity: None,
                query_tolerance: None,
                method: "LoftedWing procedural section interpolation".to_string(),
                feature_to_sdf_residual: Some(self.distance(point) as f64),
            },
        })
    }

    fn query_feature_impl(
        &self,
        point: Vec3,
        options: &ProceduralFeatureQueryOptions,
    ) -> ProceduralFeatureQueryResult {
        let Some(local) = self.local_sample(point) else {
            return ProceduralFeatureQueryResult::unsupported(
                point,
                "lofted wing local coordinates are unavailable",
            );
        };
        let mut candidates = Vec::<ProceduralFeatureMatch>::new();
        let ids:Vec<&str> = if self.smooth_sections() {
            if !self.smooth_extrusion(){return ProceduralFeatureQueryResult::unsupported(point,"smooth loft span topology is unresolved")}
            vec![TE_UPPER_ID,TE_LOWER_ID,SMOOTH_CAP_CURVES[0],SMOOTH_CAP_CURVES[1],TRAILING_CAP_CURVES[0],TRAILING_CAP_CURVES[1]]
        } else { vec![
            LE_ID,
            TE_UPPER_ID,
            TE_LOWER_ID,
            TIP_NEG_UPPER_ID,
            TIP_NEG_LOWER_ID,
            TIP_POS_UPPER_ID,
            TIP_POS_LOWER_ID,
            ROOT_SECTION_ID,
        ] };
        for id in ids {
            let parameter = if self.smooth_sections() && (SMOOTH_CAP_CURVES.contains(&id)||TRAILING_CAP_CURVES.contains(&id)) {
                let end=if id.contains(".negative."){0.0}else{1.0};
                let frame=self.section_frame_at_span_parameter(end).unwrap();
                let a=self.sections[frame.seg].section.as_any().downcast_ref::<Airfoil>().unwrap();let s=a.spline().unwrap();let q=frame.world_to_local(point).as_dvec2()/a.chord as f64;
                if SMOOTH_CAP_CURVES.contains(&id) {
                    let Ok(c)=s.closest_skin_point(q) else{return ProceduralFeatureQueryResult::unsupported(point,"unresolved spline curve coverage")};
                    match c.branch{super::naca_spline::SectionBranch::Upper=>(1.0-c.parameter)*0.5,_=>(1.0+c.parameter)*0.5}
                } else {let u=s.evaluate(true,1.0).0;let e=s.evaluate(false,1.0).0-u;((q-u).dot(e)/e.length_squared()).clamp(0.0,1.0)}
            } else if id == ROOT_SECTION_ID {
                local.airfoil_parameter
            } else if id.contains(".tip.negative.") || id.contains(".tip.positive.") {
                local.airfoil_parameter
            } else {
                local.span_parameter
            };
            if let Some(result) = self.evaluate_feature_impl(id, parameter) {
                let Some(closest_array) = result.closest_point else {
                    continue;
                };
                let closest = crate::sdf::procedural_features::array_vec3(closest_array);
                candidates.push(ProceduralFeatureMatch {
                    feature_id: id.to_string(),
                    feature_type: result.feature_type,
                    closest_point: vec3_array(closest),
                    distance: (closest - point).length() as f64,
                    tangent: result.tangent,
                    span_parameter: result.span_parameter,
                    endpoint_role: result.endpoint_role,
                    support_status: QueryValueStatus::Native,
                });
            }
        }
        candidates.sort_by(|a, b| {
            a.distance
                .total_cmp(&b.distance)
                .then_with(|| a.feature_id.cmp(&b.feature_id))
        });
        let all_matches = if options.all_features_within_tolerance {
            candidates
                .iter()
                .filter(|m| m.distance <= options.tolerance)
                .cloned()
                .collect()
        } else {
            Vec::new()
        };
        let Some(best) = candidates.first().cloned() else {
            return ProceduralFeatureQueryResult::unsupported(point, "no procedural features");
        };
        let ambiguity = candidates.get(1).and_then(|second| {
            ((second.distance - best.distance).abs() <= options.tolerance * 0.05).then(|| {
                format!(
                    "nearest procedural feature tie between {} and {}",
                    best.feature_id, second.feature_id
                )
            })
        });
        let sector = if options.surface_sector {
            self.sector_from_local(&local)
        } else {
            None
        };
        ProceduralFeatureQueryResult {
            schema_version: PROCEDURAL_FEATURE_SCHEMA_VERSION.to_string(),
            query_position: vec3_array(point),
            component_id: Some(WING_COMPONENT_ID.to_string()),
            feature_id: Some(best.feature_id.clone()),
            feature_type: best.feature_type.clone(),
            support_status: QueryValueStatus::Native,
            closest_point: Some(best.closest_point),
            distance_to_feature: Some(best.distance),
            tangent: best.tangent,
            normal_sector: sector,
            span_parameter: options.local_parameters.then_some(local.span_parameter),
            chord_parameter: options.local_parameters.then_some(local.chord_parameter),
            airfoil_parameter: options.local_parameters.then_some(local.airfoil_parameter),
            side: Some(local.side),
            cap_role: Some(local.cap_role),
            endpoint_role: best.endpoint_role.clone(),
            connected_feature_ids: self.connected_features_for(&best.feature_id),
            all_matches,
            diagnostics: ProceduralFeatureDiagnostics {
                success: true,
                warnings: Vec::new(),
                errors: Vec::new(),
                ambiguity,
                query_tolerance: Some(options.tolerance),
                method: "LoftedWing nearest procedural curve at local span/airfoil parameter"
                    .to_string(),
                feature_to_sdf_residual: Some(self.distance(
                    crate::sdf::procedural_features::array_vec3(best.closest_point),
                ) as f64),
            },
        }
    }

    fn sections_are_airfoils(&self) -> bool {
        self.sections
            .iter()
            .all(|s| s.section.as_any().downcast_ref::<Airfoil>().is_some())
    }

    fn root_span_parameter(&self) -> Option<f64> {
        let (min, max) = self.span_range()?;
        Some(((0.0 - min) / (max - min)) as f64)
    }

    fn span_range(&self) -> Option<(f32, f32)> {
        let min = self
            .sections
            .iter()
            .map(|s| s.position.y)
            .fold(f32::INFINITY, f32::min);
        let max = self
            .sections
            .iter()
            .map(|s| s.position.y)
            .fold(f32::NEG_INFINITY, f32::max);
        (min.is_finite() && max.is_finite() && max > min).then_some((min, max))
    }

    fn section_pair_at_span_parameter(&self, span_parameter: f64) -> Option<(usize, f32)> {
        let (min, max) = self.span_range()?;
        let y = min + (max - min) * span_parameter.clamp(0.0, 1.0) as f32;
        self.section_pair_at_y(y).map(|(i, t, _)| (i, t))
    }

    fn section_pair_at_y(&self, y: f32) -> Option<(usize, f32, f32)> {
        if self.sections.len() < 2 {
            return None;
        }
        let (min, max) = self.span_range()?;
        let y_clamped = y.clamp(min, max);
        let mut seg_idx = self.sections.len() - 2;
        let mut t = 1.0f32;
        for i in 0..self.sections.len() - 1 {
            let ya = self.sections[i].position.y;
            let yb = self.sections[i + 1].position.y;
            let span = yb - ya;
            if span.abs() < 1e-6 {
                continue;
            }
            let raw = (y_clamped - ya) / span;
            if (0.0..=1.0).contains(&raw) {
                seg_idx = i;
                t = raw;
                break;
            }
        }
        Some((seg_idx, t, y_clamped))
    }

    fn section_frame_at_span_parameter(&self, span_parameter: f64) -> Option<WingFrame> {
        let (seg, t) = self.section_pair_at_span_parameter(span_parameter)?;
        Some(self.frame_for_segment(seg, t))
    }

    fn frame_for_segment(&self, seg: usize, t: f32) -> WingFrame {
        let a = &self.sections[seg];
        let b = &self.sections[seg + 1];
        WingFrame {
            leading_edge: a.position + (b.position - a.position) * t,
            twist: a.twist + (b.twist - a.twist) * t,
            seg,
            t,
        }
    }

    fn local_airfoil_point(&self, seg: usize, t: f32, p: AirfoilFeaturePoint) -> Option<Vec2> {
        let a = self.sections[seg]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let b = self.sections[seg + 1]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let pa = airfoil_feature_point(a, p)?;
        let pb = airfoil_feature_point(b, p)?;
        let norm = pa * (1.0 - t) + pb * t;
        let chord = a.chord + (b.chord - a.chord) * t;
        Some(norm * chord)
    }

    fn span_curve_point(&self, span_parameter: f64, p: AirfoilFeaturePoint) -> Option<Vec3> {
        let frame = self.section_frame_at_span_parameter(span_parameter)?;
        let local = self.local_airfoil_point(frame.seg, frame.t, p)?;
        Some(frame.local_to_world(local))
    }

    fn span_curve_tangent(&self, span_parameter: f64, p: AirfoilFeaturePoint) -> Option<Vec3> {
        let h = 1.0e-4;
        let a = self.span_curve_point((span_parameter - h).max(0.0), p)?;
        let b = self.span_curve_point((span_parameter + h).min(1.0), p)?;
        Some((b - a).normalize_or_zero())
    }

    fn tip_curve_point(&self, positive: bool, upper: bool, parameter: f64) -> Option<Vec3> {
        let span = if positive { 1.0 } else { 0.0 };
        let frame = self.section_frame_at_span_parameter(span)?;
        let (seg, t) = (frame.seg, frame.t);
        let local = self.branch_point(seg, t, upper, parameter)?;
        Some(frame.local_to_world(local))
    }

    fn tip_curve_tangent(&self, positive: bool, upper: bool, parameter: f64) -> Option<Vec3> {
        let h = 1.0e-4;
        let a = self.tip_curve_point(positive, upper, (parameter - h).max(0.0))?;
        let b = self.tip_curve_point(positive, upper, (parameter + h).min(1.0))?;
        Some((b - a).normalize_or_zero())
    }

    fn root_section_point(&self, parameter: f64) -> Option<Vec3> {
        let span = self.root_span_parameter()?;
        let frame = self.section_frame_at_span_parameter(span)?;
        let local = self.branch_point(
            frame.seg,
            frame.t,
            parameter < 0.5,
            if parameter < 0.5 {
                parameter * 2.0
            } else {
                (parameter - 0.5) * 2.0
            },
        )?;
        Some(frame.local_to_world(local))
    }

    fn root_section_tangent(&self, parameter: f64) -> Option<Vec3> {
        let h = 1.0e-4;
        let a = self.root_section_point((parameter - h).max(0.0))?;
        let b = self.root_section_point((parameter + h).min(1.0))?;
        Some((b - a).normalize_or_zero())
    }

    fn branch_point(&self, seg: usize, t: f32, upper: bool, parameter: f64) -> Option<Vec2> {
        let a = self.sections[seg]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let b = self.sections[seg + 1]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let pa = airfoil_branch_point(a, upper, parameter)?;
        let pb = airfoil_branch_point(b, upper, parameter)?;
        let norm = pa * (1.0 - t) + pb * t;
        let chord = a.chord + (b.chord - a.chord) * t;
        Some(norm * chord)
    }

    fn local_sample(&self, point: Vec3) -> Option<WingLocalSample> {
        let (seg, t, y_clamped) = self.section_pair_at_y(point.y)?;
        let frame = self.frame_for_segment(seg, t);
        let local = frame.world_to_local(point);
        let a = self.sections[seg]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let b = self.sections[seg + 1]
            .section
            .as_any()
            .downcast_ref::<Airfoil>()?;
        let chord = a.chord + (b.chord - a.chord) * t;
        let normalized = local / chord.max(1.0e-6);
        let nearest = nearest_lerped_airfoil_segment(a, b, t, normalized)?;
        let (span_min, span_max) = self.span_range()?;
        let span_parameter = ((y_clamped - span_min) / (span_max - span_min)) as f64;
        let trailing_edge_band = self
            .feature_metadata()
            .map(|metadata| {
                let half_thickness = 0.5 * metadata.trailing_edge_thickness_mm / chord.max(1.0e-6);
                normalized.x > 0.96 && normalized.y.abs() <= (half_thickness * 1.75).max(0.02)
            })
            .unwrap_or(false);
        let side = if !self.smooth_sections() && trailing_edge_band {
            WingSurfaceSide::Cap
        } else {
            nearest.side
        };
        let cap_role = if self.smooth_sections() {
            let profile=a.lerped_distance_inline(b,t,local);
            let span_distance=(span_min-point.y).max(point.y-span_max);
            if span_distance>=profile {if point.y<=(span_min+span_max)*0.5{CapRole::TipCapNegative}else{CapRole::TipCapPositive}}
            else if side==WingSurfaceSide::Cap{CapRole::TrailingEdgeCap}else{CapRole::None}
        } else if (point.y - span_min).abs() < 1.0 {
            CapRole::TipCapNegative
        } else if (point.y - span_max).abs() < 1.0 {
            CapRole::TipCapPositive
        } else if side == WingSurfaceSide::Cap {
            CapRole::TrailingEdgeCap
        } else {
            CapRole::None
        };
        Some(WingLocalSample {
            span_parameter,
            chord_parameter: normalized.x.clamp(0.0, 1.0) as f64,
            airfoil_parameter: nearest.parameter,
            side,
            cap_role,
        })
    }

    fn sector_for_side(&self, side: WingSurfaceSide, cap_role: CapRole) -> Option<SurfaceSectorId> {
        if self.smooth_sections() && matches!(cap_role,CapRole::None) && matches!(side,WingSurfaceSide::Upper|WingSurfaceSide::Lower) {
            return Some(SurfaceSectorId{stable_id:SKIN_ID.into(),support_status:QueryValueStatus::Native})
        }
        let id = match cap_role {
            CapRole::TrailingEdgeCap => TE_CAP_ID,
            CapRole::TipCapNegative => TIP_NEG_SECTOR_ID,
            CapRole::TipCapPositive => TIP_POS_SECTOR_ID,
            _ => match side {
                WingSurfaceSide::Upper => UPPER_SECTOR_ID,
                WingSurfaceSide::Lower => LOWER_SECTOR_ID,
                WingSurfaceSide::Cap => TE_CAP_ID,
                WingSurfaceSide::Unknown => return None,
            },
        };
        Some(SurfaceSectorId {
            stable_id: id.to_string(),
            support_status: QueryValueStatus::Native,
        })
    }

    fn sector_from_local(&self, local: &WingLocalSample) -> Option<SurfaceSectorId> {
        self.sector_for_side(local.side, local.cap_role)
    }

    fn connected_features_for(&self, feature_id: &str) -> Vec<String> {
        self.procedural_feature_inventory_impl()
            .and_then(|inventory| {
                inventory
                    .features
                    .into_iter()
                    .find(|f| f.feature_id == feature_id)
                    .map(|f| f.connected_feature_ids)
            })
            .unwrap_or_default()
    }

    fn chord_parameter_for_feature(&self, feature_id: &str, parameter: f64) -> Option<f64> {
        match feature_id {
            LE_ID => Some(0.0),
            TE_UPPER_ID | TE_LOWER_ID => Some(1.0),
            ROOT_SECTION_ID => Some(parameter),
            id if id.contains(".tip.") => Some(parameter),
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
struct WingFrame {
    leading_edge: Vec3,
    twist: f32,
    seg: usize,
    t: f32,
}

impl WingFrame {
    fn local_to_world(&self, local: Vec2) -> Vec3 {
        let (sin_t, cos_t) = self.twist.sin_cos();
        Vec3::new(
            self.leading_edge.x + local.x * cos_t - local.y * sin_t,
            self.leading_edge.y,
            self.leading_edge.z + local.x * sin_t + local.y * cos_t,
        )
    }

    fn world_to_local(&self, point: Vec3) -> Vec2 {
        let local = Vec2::new(point.x - self.leading_edge.x, point.z - self.leading_edge.z);
        let (sin_t, cos_t) = (-self.twist).sin_cos();
        Vec2::new(
            local.x * cos_t - local.y * sin_t,
            local.x * sin_t + local.y * cos_t,
        )
    }
}

#[derive(Clone, Copy)]
struct WingLocalSample {
    span_parameter: f64,
    chord_parameter: f64,
    airfoil_parameter: f64,
    side: WingSurfaceSide,
    cap_role: CapRole,
}

#[derive(Clone, Copy)]
enum AirfoilFeaturePoint {
    Leading,
    TrailingUpper,
    TrailingLower,
}

#[derive(Clone, Copy)]
struct NearestAirfoilSegment {
    side: WingSurfaceSide,
    parameter: f64,
}

fn feature_definition(
    id: &str,
    feature_type: ProceduralFeatureType,
    endpoint_roles: Vec<FeatureEndpointRole>,
    connected: Vec<&str>,
    curve: bool,
    finite_te: Option<f64>,
) -> ProceduralFeatureDefinition {
    ProceduralFeatureDefinition {
        feature_id: id.to_string(),
        feature_type,
        support_status: QueryValueStatus::Native,
        curve_domain: curve.then_some([0.0, 1.0]),
        closed: false,
        endpoint_roles,
        connected_feature_ids: connected.into_iter().map(str::to_string).collect(),
        source: "LoftedWing procedural section interpolation".to_string(),
        available_parameters: vec![
            "normalized_span".to_string(),
            "normalized_chord_or_branch".to_string(),
        ],
        tangent_available: true,
        normal_sector_available: true,
        known_singularities: Vec::new(),
        cap_exists: Some(matches!(
            id,
            TE_UPPER_ID
                | TE_LOWER_ID
                | TIP_NEG_UPPER_ID
                | TIP_NEG_LOWER_ID
                | TIP_POS_UPPER_ID
                | TIP_POS_LOWER_ID
        )),
        finite_trailing_edge_thickness: finite_te,
    }
}

fn sector(
    id: &str,
    sector_type: ProceduralFeatureType,
    adjacent: Vec<&str>,
) -> SurfaceSectorDefinition {
    SurfaceSectorDefinition {
        sector_id: id.to_string(),
        sector_type,
        support_status: QueryValueStatus::Native,
        adjacent_feature_ids: adjacent.into_iter().map(str::to_string).collect(),
        notes: Vec::new(),
    }
}

fn junction(
    id: &str,
    role: FeatureEndpointRole,
    connected: Vec<&str>,
    span: Option<f64>,
) -> ProceduralFeatureJunction {
    ProceduralFeatureJunction {
        junction_id: id.to_string(),
        role,
        connected_feature_ids: connected.into_iter().map(str::to_string).collect(),
        span_parameter: span,
    }
}

fn span_endpoint(parameter: f64) -> Option<FeatureEndpointRole> {
    if parameter <= 1.0e-8 {
        Some(FeatureEndpointRole::NegativeTip)
    } else if parameter >= 1.0 - 1.0e-8 {
        Some(FeatureEndpointRole::PositiveTip)
    } else {
        Some(FeatureEndpointRole::Interior)
    }
}

fn tip_endpoint(parameter: f64, upper: bool) -> Option<FeatureEndpointRole> {
    if parameter <= 1.0e-8 {
        Some(FeatureEndpointRole::LeadingTip)
    } else if parameter >= 1.0 - 1.0e-8 {
        Some(if upper {
            FeatureEndpointRole::TrailingUpperTip
        } else {
            FeatureEndpointRole::TrailingLowerTip
        })
    } else {
        Some(FeatureEndpointRole::Interior)
    }
}

fn airfoil_feature_point(airfoil: &Airfoil, p: AirfoilFeaturePoint) -> Option<Vec2> {
    if let Some(s)=airfoil.spline(){return Some(match p{AirfoilFeaturePoint::Leading=>s.evaluate(true,0.0).0,AirfoilFeaturePoint::TrailingUpper=>s.evaluate(true,1.0).0,AirfoilFeaturePoint::TrailingLower=>s.evaluate(false,1.0).0}.as_vec2())}
    match p {
        AirfoilFeaturePoint::TrailingUpper => {
            airfoil.points.first().map(|(x, y)| Vec2::new(*x, *y))
        }
        AirfoilFeaturePoint::TrailingLower => airfoil
            .points
            .get(airfoil.points.len().saturating_sub(2))
            .map(|(x, y)| Vec2::new(*x, *y)),
        AirfoilFeaturePoint::Leading => airfoil
            .points
            .iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(x, y)| Vec2::new(*x, *y)),
    }
}

fn airfoil_branch_point(airfoil: &Airfoil, upper: bool, parameter: f64) -> Option<Vec2> {
    if let Some(s)=airfoil.spline(){return Some(s.evaluate(upper,parameter).0.as_vec2())}
    let leading_idx = airfoil
        .points
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.0.total_cmp(&b.0))
        .map(|(i, _)| i)?;
    let parameter = parameter.clamp(0.0, 1.0);
    if upper {
        let idx = ((leading_idx as f64) * (1.0 - parameter)).round() as usize;
        airfoil.points.get(idx).map(|(x, y)| Vec2::new(*x, *y))
    } else {
        let lower_end = airfoil.points.len().saturating_sub(2);
        let idx = leading_idx + ((lower_end - leading_idx) as f64 * parameter).round() as usize;
        airfoil.points.get(idx).map(|(x, y)| Vec2::new(*x, *y))
    }
}

fn nearest_lerped_airfoil_segment(
    a: &Airfoil,
    b: &Airfoil,
    t: f32,
    point: Vec2,
) -> Option<NearestAirfoilSegment> {
    if let (Some(sa),Some(sb))=(a.spline(),b.spline()) {
        let blended;let s=if sa==sb {sa.as_ref()}else{blended=sa.blended(sb,t as f64)?;&blended};
        let q=s.closest_point(point.as_dvec2()).ok()?;
        let side=match q.branch{super::naca_spline::SectionBranch::Upper=>WingSurfaceSide::Upper,super::naca_spline::SectionBranch::Lower=>WingSurfaceSide::Lower,super::naca_spline::SectionBranch::TrailingFace=>WingSurfaceSide::Cap};
        return Some(NearestAirfoilSegment{side,parameter:q.parameter});
    }
    let n = a.points.len().min(b.points.len());
    if n < 2 {
        return None;
    }
    let leading_idx = a
        .points
        .iter()
        .enumerate()
        .min_by(|(_, x), (_, y)| x.0.total_cmp(&y.0))
        .map(|(i, _)| i)?;
    let s = 1.0 - t;
    let mut best = (f32::INFINITY, 0usize, 0.0f32);
    for i in 0..n - 1 {
        let p0 = Vec2::new(
            a.points[i].0 * s + b.points[i].0 * t,
            a.points[i].1 * s + b.points[i].1 * t,
        );
        let p1 = Vec2::new(
            a.points[i + 1].0 * s + b.points[i + 1].0 * t,
            a.points[i + 1].1 * s + b.points[i + 1].1 * t,
        );
        let edge = p1 - p0;
        let denom = edge.length_squared();
        let u = if denom > 1.0e-12 {
            ((point - p0).dot(edge) / denom).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let d2 = (point - (p0 + edge * u)).length_squared();
        if d2 < best.0 {
            best = (d2, i, u);
        }
    }
    let side = if best.1 == n - 2 {
        WingSurfaceSide::Cap
    } else if best.1 < leading_idx {
        WingSurfaceSide::Upper
    } else {
        WingSurfaceSide::Lower
    };
    let parameter = match side {
        WingSurfaceSide::Upper => 1.0 - (best.1 as f64 + best.2 as f64) / leading_idx.max(1) as f64,
        WingSurfaceSide::Lower => {
            let denom = (n - 2).saturating_sub(leading_idx).max(1) as f64;
            (best.1.saturating_sub(leading_idx) as f64 + best.2 as f64) / denom
        }
        WingSurfaceSide::Cap => best.2 as f64,
        WingSurfaceSide::Unknown => 0.0,
    }
    .clamp(0.0, 1.0);
    Some(NearestAirfoilSegment { side, parameter })
}

/// Create a parametric wing with full geometric control
///
/// # Arguments
/// * `airfoil_designation` - NACA code (e.g., "2412")
/// * `root_chord` - Chord length at wing root
/// * `tip_chord` - Chord length at wing tip
/// * `span` - Total wing span (tip to tip)
/// * `sweep` - Leading edge sweep angle in degrees (positive = swept back)
/// * `dihedral` - Dihedral angle in degrees (positive = tips higher than root)
/// * `twist` - Tip twist relative to root in degrees (negative = washout)
pub fn wing_with_airfoil(
    airfoil_designation: &str,
    root_chord: f32,
    tip_chord: f32,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
) -> LoftedWing {
    let root_airfoil = get_naca_airfoil(airfoil_designation, root_chord);
    let tip_airfoil = get_naca_airfoil(airfoil_designation, tip_chord);
    wing_from_airfoil_sections(root_airfoil, tip_airfoil, span, sweep, dihedral, twist)
}

pub fn wing_with_airfoil_export_safe(
    airfoil_designation: &str,
    root_chord: f32,
    tip_chord: f32,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
    export_options: AirfoilExportOptions,
) -> LoftedWing {
    let root_airfoil =
        get_naca_airfoil_export_safe(airfoil_designation, root_chord, export_options);
    let tip_airfoil = get_naca_airfoil_export_safe(airfoil_designation, tip_chord, export_options);
    wing_from_airfoil_sections(root_airfoil, tip_airfoil, span, sweep, dihedral, twist)
}

fn wing_from_airfoil_sections(
    root_airfoil: Arc<Airfoil>,
    tip_airfoil: Arc<Airfoil>,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
) -> LoftedWing {
    let sweep_rad = sweep.to_radians();
    let dihedral_rad = dihedral.to_radians();
    let twist_rad = twist.to_radians();
    let half_span = span / 2.0;

    let root_section = WingSection {
        section: root_airfoil,
        position: Vec3::ZERO,
        twist: 0.0,
    };

    // Span along Y; sweep moves leading edge back in X; dihedral lifts tips in Z.
    let tip_x = half_span * sweep_rad.tan();
    let tip_y = half_span * dihedral_rad.cos();
    let tip_z = half_span * dihedral_rad.sin();

    let tip_section = WingSection {
        section: tip_airfoil,
        position: Vec3::new(tip_x, tip_y, tip_z),
        twist: twist_rad,
    };

    // Mirror tip in Y for the opposite semi-span
    let neg_tip_section = WingSection {
        section: tip_section.section.clone(),
        position: Vec3::new(tip_x, -tip_y, tip_z),
        twist: tip_section.twist,
    };

    // Sections ordered by Y: neg_tip → root → pos_tip
    LoftedWing::new(vec![neg_tip_section, root_section, tip_section])
}

/// Create a wing from pre-built Section2D root and tip cross-sections.
///
/// The chord length is encoded in the sections themselves (e.g. via `Airfoil::chord`).
/// Use this overload when you have custom airfoil data loaded from points or a .dat file.
///
/// # Arguments
/// * `root_section` - Cross-section at the wing root; chord is baked in.
/// * `tip_section`  - Cross-section at the wing tip; chord is baked in.
/// * `span`         - Total wing span (tip to tip)
/// * `sweep`        - Leading edge sweep angle in degrees
/// * `dihedral`     - Dihedral angle in degrees
/// * `twist`        - Tip twist relative to root in degrees
pub fn wing_from_sections(
    root_section: Arc<dyn Section2D>,
    tip_section: Arc<dyn Section2D>,
    span: f32,
    sweep: f32,
    dihedral: f32,
    twist: f32,
) -> LoftedWing {
    let sweep_rad = sweep.to_radians();
    let dihedral_rad = dihedral.to_radians();
    let twist_rad = twist.to_radians();
    let half_span = span / 2.0;

    let root = WingSection {
        section: root_section,
        position: Vec3::ZERO,
        twist: 0.0,
    };

    let tip_x = half_span * sweep_rad.tan();
    let tip_y = half_span * dihedral_rad.cos();
    let tip_z = half_span * dihedral_rad.sin();

    let tip = WingSection {
        section: tip_section.clone(),
        position: Vec3::new(tip_x, tip_y, tip_z),
        twist: twist_rad,
    };

    let neg_tip = WingSection {
        section: tip_section,
        position: Vec3::new(tip_x, -tip_y, tip_z),
        twist: twist_rad,
    };

    LoftedWing::new(vec![neg_tip, root, tip])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unresolved_spline_section_fails_closed_through_wing_caps() {
        let section = get_naca_airfoil_export_safe("0000", 1.0, AirfoilExportOptions {
            min_trailing_edge_thickness_mm: 0.0,
            min_leading_edge_radius_mm: 0.0,
            clip_trailing_edge: false,
        });
        let wing = LoftedWing::new(vec![
            WingSection { section: section.clone(), position: Vec3::new(0.0, -0.05, 0.0), twist: 0.0 },
            WingSection { section, position: Vec3::new(0.0, 0.05, 0.0), twist: 0.0 },
        ]);
        assert!(wing.distance(Vec3::new(0.3, 0.0, 0.0)).is_nan());
        assert!(wing.distance(Vec3::new(0.3, f32::NAN, 0.0)).is_nan());
    }
    use crate::sdf::conditioning::{BoundQuality, assert_conditionable_geometry};

    #[test]
    fn test_straight_wing() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        assert_eq!(wing.sections[1].position, Vec3::ZERO);
        assert!((wing.sections[0].position.y + 15.0).abs() < 0.01);
        assert!((wing.sections[2].position.y - 15.0).abs() < 0.01);
        assert!(wing.sections[2].position.x.abs() < 0.01);
        assert!(wing.sections[2].position.z.abs() < 0.01);
    }

    #[test]
    fn lofted_wing_metadata_tracks_airfoil_section_bounds() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);
        let metadata = wing.metadata();
        let bounds = metadata
            .support_bounds
            .as_ref()
            .expect("wing should expose live support bounds");

        assert!(bounds.min.y <= -15.0);
        assert!(bounds.max.y >= 15.0);
        assert!(bounds.max.x >= 10.0);
        assert!(metadata.bounds_are_approximate);
        assert_eq!(metadata.bound_quality, BoundQuality::Conservative);
        assert!(assert_conditionable_geometry(&metadata).is_ok());
    }

    #[test]
    fn export_safe_lofted_wing_metadata_reports_feature_regions() {
        let wing = wing_with_airfoil_export_safe(
            "0012",
            80.0,
            40.0,
            600.0,
            5.0,
            3.0,
            -2.0,
            AirfoilExportOptions {
                min_trailing_edge_thickness_mm: 1.5,
                min_leading_edge_radius_mm: 1.0,
                clip_trailing_edge: true,
            },
        );

        let metadata = wing.metadata();

        assert_eq!(metadata.min_feature_size, Some(1.5));
        assert_eq!(metadata.feature_regions.len(), 2);
        assert!(
            metadata
                .feature_regions
                .iter()
                .all(|region| region.bounds.size().y > 500.0)
        );
    }

    #[test]
    fn test_swept_wing() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 30.0, 0.0, 0.0);

        let tip = &wing.sections[2];
        assert!(
            tip.position.x > 0.0,
            "Swept wing should have positive X offset"
        );

        let expected_x = (30.0_f32.to_radians()).tan() * 15.0;
        assert!((tip.position.x - expected_x).abs() < 0.1);
    }

    #[test]
    fn test_dihedral() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 5.0, 0.0);

        let tip = &wing.sections[2];
        assert!(
            tip.position.z > 0.0,
            "Dihedral wing tip should have positive Z offset"
        );

        let expected_z = (5.0_f32.to_radians()).sin() * 15.0;
        assert!((tip.position.z - expected_z).abs() < 0.1);
    }

    #[test]
    fn test_twist() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, -3.0);

        let tip = &wing.sections[2];
        let expected_twist = (-3.0_f32).to_radians();
        assert!((tip.twist - expected_twist).abs() < 0.01);
    }

    #[test]
    fn test_wing_sdf_evaluation() {
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        let _ = wing.distance(Vec3::new(5.0, 0.0, 0.0));

        let dist_outside = wing.distance(Vec3::new(100.0, 100.0, 100.0));
        assert!(
            dist_outside > 50.0,
            "Point far outside wing should have large positive distance"
        );

        let dist_beyond = wing.distance(Vec3::new(5.0, 0.0, 50.0));
        assert!(
            dist_beyond > 10.0,
            "Point beyond span should be far outside"
        );
    }

    #[test]
    fn test_lofted_interpolation_uses_both_sections() {
        // Root chord=10, tip chord=5. At mid-span (t=0.5), interpolated chord ≈ 7.5.
        // A point at x=9 (beyond tip chord, within root chord) should be outside
        // at mid-span but inside near the root.
        let wing = wing_with_airfoil("0012", 10.0, 5.0, 30.0, 0.0, 0.0, 0.0);

        let near_root = wing.distance(Vec3::new(9.0, 1.0, 0.0)); // y=1, near root
        let near_tip = wing.distance(Vec3::new(9.0, 14.0, 0.0)); // y=14, near tip

        // Near root the chord is ~10 so x=9 should be within the airfoil silhouette
        assert!(
            near_root < near_tip,
            "x=9 should be closer to surface near root (larger chord) than near tip (smaller chord)"
        );
    }
}

#[test]
fn test_wing_interior_points() {
    use crate::sdf::Sdf;
    let wing = wing_with_airfoil("2412", 12.0, 5.0, 40.0, 20.0, 3.0, -4.0);
    // Root section: chord=12, leading edge at (0,0,0), span at y=0
    // X=6 is 50% chord, Z=0 is near center, Y=0 is root
    let d_root = wing.distance(Vec3::new(6.0, 0.0, 0.0));
    println!("root mid-chord (6,0,0): {}", d_root);
    // X=10, Y=0: 83% chord at root — inside
    let d_root2 = wing.distance(Vec3::new(10.0, 0.0, 0.0));
    println!("root 83% chord (10,0,0): {}", d_root2);
    // Span-aligned: Y=10
    let d_mid = wing.distance(Vec3::new(5.0, 10.0, 0.5));
    println!("mid-span (5,10,0.5): {}", d_mid);
    assert!(
        d_root < 0.0,
        "root mid-chord should be inside, got {}",
        d_root
    );
    assert!(
        d_root2 < 0.0,
        "root 83% chord should be inside, got {}",
        d_root2
    );
}
