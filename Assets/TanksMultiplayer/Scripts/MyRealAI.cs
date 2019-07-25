using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.SceneManagement;

namespace TanksMP
{
    //TODO: bullet dodging
    //TODO: cover mechanism
    //TODO: powerup pickup
    //TODO: precise aiming
    //TODO: maintain distance
    //TODO: do not use straight line dist as criteria
    //TODO: remember all spawned items
    public class MyRealAI : BaseControl
    {
        private enum AIState
        {
            DodgeAndAttack,
            ActivelyAttack,
            FindPowerup,
            SimpleGoToPowerup,
        }

        private enum MotionType
        {
            Linear,
            NavMeshAgent,
        }

        private struct MotionInfo
        {
            public MotionInfo(MotionType motionType, Vector3 position, Vector3 velocity, Vector3 boundSize, NavMeshAgent agent = null)
            {
                this.motionType = motionType;
                this.position = position;
                this.velocity = velocity;

                collider = null;
                bounds = new Bounds(Vector3.zero, boundSize);

                this.agent = agent;
            }

            public MotionInfo(MotionType motionType, Vector3 position, Vector3 velocity, GameObject entity)
            {
                this.motionType = motionType;
                this.position = position;
                this.velocity = velocity;
                collider = entity.GetComponent<Collider>();

                if (!collider)
                {
                    Debug.LogWarning("no collider is found on " + entity.name);
                }

                if (motionType == MotionType.NavMeshAgent)
                {
                    agent = entity.GetComponent<NavMeshAgent>();
                    if (!agent) Debug.LogWarning("no agent is found on " + entity.name);
                }
                else
                {
                    agent = null;
                }

                bounds = default(Bounds);
            }

            public MotionType motionType;
            //initial motion status
            public Vector3 position;
            public Vector3 velocity;
            
            public Bounds bounds;

            //comp reference
            private Collider collider;
            private NavMeshAgent agent;

            public Bounds PredictPosition(int timeStep)
            {
                //update AABB box position
                if (collider)
                {
                    bounds = collider.bounds;
                }
                else
                {
                    bounds.center = agent ? agent.transform.position : position;
                    //Vector3 center = bounds.center;
                    //center.y += bounds.size.y / 2.0f;
                    //bounds.center = center;
                }

                //do computation
                if (timeStep == 0) return bounds;
                switch(motionType)
                {
                    case MotionType.Linear:
                        {
                            Bounds ret = bounds;
                            if (Mathf.Approximately(velocity.sqrMagnitude, 0.0f))
                            {
                                return ret;
                            }
                            
                            Vector3 displacement = velocity * Time.fixedDeltaTime * timeStep;
                            Vector3 tempPos = bounds.center + displacement;

                            //Debug.DrawLine(position, tempPos, Color.cyan);

                            Vector3 obstaclePos;
                            if(CheckObstaclesBetween(bounds.center, tempPos, out obstaclePos, bounds.size))
                            {
                                ret.center = obstaclePos;
                                return ret;
                            }
                            else
                            {
                                displacement.y = 0;
                                ret.center += displacement;
                                return ret;
                            }
                        }

                    case MotionType.NavMeshAgent:
                        {
                            Bounds ret = bounds;
                            if(agent.path.status == NavMeshPathStatus.PathInvalid)
                            {
                                goto case MotionType.Linear;
                            }

                            Vector3[] corners = agent.path.corners;

                            float minDist = Mathf.Infinity;
                            int minIndex = -1;
                            int i;
                            for(i = 0; i < corners.Length; i++)
                            {
                                float dist = Vector3.Distance(corners[i], agent.steeringTarget);
                                if (minDist > dist)
                                {
                                    minDist = dist;
                                    minIndex = i;
                                }
                            }
                            if (minIndex == -1) Debug.Break();

                            i = minIndex;
                            
                            float accumulatedSteps = 0f;
                            float stepsToCorner, stepsToTurn;

                            Vector3 pos = position;
                            Vector3 finalPos;
                            Vector3 displacement;
                            while (i < corners.Length)
                            {
                                //calculate time it takes to reach the steering target
                                stepsToCorner = Vector3.Distance(corners[i], pos) / (agent.speed * Time.fixedDeltaTime);

                                if (accumulatedSteps + stepsToCorner > timeStep)
                                {
                                    finalPos = Vector3.Lerp(pos, corners[i], (timeStep - accumulatedSteps) / stepsToCorner);

                                    displacement = finalPos - position;
                                    displacement.y = 0;
                                    ret.center += displacement;

                                    return ret;
                                }

                                if (i + 1 == corners.Length) break;

                                //calculate time it takes to turn
                                Vector3 desiredDir = corners[i+1] - corners[i];
                                float angle = Vector3.Angle(velocity, desiredDir);
                                stepsToTurn = angle / (agent.angularSpeed * Time.fixedDeltaTime);

                                if (accumulatedSteps + stepsToCorner + stepsToTurn > timeStep)
                                {
                                    finalPos = corners[i];

                                    displacement = finalPos - position;
                                    displacement.y = 0;
                                    ret.center += displacement;

                                    return ret;
                                }

                                pos = corners[i];
                                i++;
                                accumulatedSteps += (stepsToCorner + stepsToTurn);
                            }

                            finalPos = agent.destination;

                            displacement = finalPos - position;
                            displacement.y = 0;
                            ret.center += displacement;

                            return ret;
                        }

                    default:
                        {
                            Debug.LogWarning("unknown motion type");
                            return default(Bounds);
                        }
                }
            }
        }

        [SerializeField] private float m_PredictionTime = 0.5f; //in sec
        private int m_PredictionStep; //how many frames for the prediction time

        [SerializeField] private float m_BulletSensorRadius = 15f;
        [SerializeField] private float m_PlayerSensorRadius = 35f;
        [SerializeField] private float m_ItemSensorRadius = 50f;
        [SerializeField] private float m_CombatDist = 6f;
        [SerializeField] private float m_EscapeDist = 10f;

        //stats
        private float m_BulletSpeed = 10;
        private float m_TankSpeed = 8;
        private float m_BulletRadius = 0.2f;
        private int m_ShieldAmount = 3;

        private Bullet m_Threat;
        private Bullet m_ThreatLastFrame;

        private BasePlayer m_Target;
        private ObjectSpawner m_PowerUp;

        private AIState m_CurState;
        private AIState m_PrevState;

        private MotionInfo m_PlayerMotionInfo;
        private BoxCollider m_Collider;

        private ObjectSpawner m_SheildSpawner;
        private bool m_HasSheild, m_HasShieldLastFrame;
        private float m_LastPickupTime;
        private float m_NextSpawnTime;

        private NavMeshPath m_PathBuffer;

        //debug
        Bullet[] bullets;
        BasePlayer[] enemys;

        protected override void OnInit()
        {
            base.OnInit();
            m_Collider = tankPlayer.GetComponent<BoxCollider>();
            ChangeState(AIState.FindPowerup);
            m_PlayerMotionInfo = new MotionInfo(MotionType.NavMeshAgent, tankPlayer.Position, tankPlayer.Velocity, tankPlayer.gameObject);
            m_PredictionStep = Mathf.RoundToInt(m_PredictionTime / Time.fixedDeltaTime);
            m_PathBuffer = new NavMeshPath();
        }

        protected override void OnFixedUpdate()
        {
            base.OnFixedUpdate();

            if (m_SheildSpawner)
            {
                m_HasSheild = m_SheildSpawner.obj;

                if (m_HasShieldLastFrame && !m_HasSheild)
                {
                    m_LastPickupTime = Time.time;
                    m_NextSpawnTime = m_LastPickupTime + 15f;
                }

                m_HasShieldLastFrame = m_HasSheild;
            }
        }

        protected override void OnUpdate()
        {
            base.OnUpdate();

            m_ThreatLastFrame = m_Threat;
            m_Threat = null;

            m_PlayerMotionInfo.position = tankPlayer.Position;
            m_PlayerMotionInfo.velocity = tankPlayer.Velocity;

            bullets = CheckSurroundingObjectsByType(m_BulletSensorRadius,
                (Bullet bullet) => !CheckObstaclesBetween(bullet.Position, tankPlayer.Position, 0f) && bullet.owner != tankPlayer.gameObject);

            if (bullets.Length > 0)
            {
                float minDist = Mathf.Infinity;
                int minIndex = -1;
                for (int i = 0; i < bullets.Length; i++)
                {
                    Vector3 bulletToPlayer = tankPlayer.Position - bullets[i].Position;
                    bulletToPlayer = Vector3.ProjectOnPlane(bulletToPlayer, Vector3.up);
                    Vector3 bulletVelocity = Vector3.ProjectOnPlane(bullets[i].Velocity, Vector3.up);

                    float dist = Vector3.Distance(bullets[i].Position, tankPlayer.Position);

                    if (Vector3.Angle(bulletToPlayer, bulletVelocity) > 30 && dist >= 3.5f)
                    {
                        continue;
                    }

                    if (minDist > dist)
                    {
                        minDist = dist;
                        minIndex = i;
                    }
                }
                if (minIndex != -1)
                {
                    m_Threat = bullets[minIndex];
                    ChangeState(AIState.DodgeAndAttack);
                }
            }

            if (m_Target && !CheckObstaclesBetween(m_Target.shotPos.position, tankPlayer.shotPos.position, m_BulletRadius))
            {
                //aim
                PreciseShootTarget();
            }

            switch (m_CurState)
            {
                case AIState.DodgeAndAttack:
                    {
                        DodgeAttackUpdate();
                        break;
                    }
                case AIState.ActivelyAttack:
                    {
                        ActiveAttackUpdate();
                        break;
                    }
                case AIState.FindPowerup:
                    {
                        FindPowerupUpdate();
                        break;
                    }
                case AIState.SimpleGoToPowerup:
                    {
                        SimpleGoToPowerupUpdate();
                        break;
                    }
            }
        }

        private void ChangeState(AIState newState)
        {
            ResumeAgentControl();

            if (newState != m_CurState)
            {
                m_PrevState = m_CurState;
            }

            m_CurState = newState;
        }

        private void ResumeAgentControl()
        {
            //reset agent
            tankPlayer.agent.isStopped = false;
            tankPlayer.agent.updateRotation = true;
        }
        
        private void ManualControl(bool clearPath)
        {
            if(clearPath)
                tankPlayer.agent.ResetPath();

            tankPlayer.agent.isStopped = true;
            tankPlayer.agent.updateRotation = false;
        }

        private void DodgeAttackUpdate()
        {
            ManualControl(false);

            if (!m_Threat)
            {
                ChangeState(m_PrevState);
                return;
            }

            Dodge();
        }

        private void SimpleGoToPowerupUpdate()
        {
            if (!m_PowerUp || (m_PowerUp != m_SheildSpawner && !m_PowerUp.obj))
            {
                ChangeState(AIState.ActivelyAttack);
                return;
            }

            if (tankPlayer.agent.remainingDistance <= tankPlayer.agent.stoppingDistance && !tankPlayer.agent.pathPending)
            {
                ChangeState(AIState.ActivelyAttack);
                return;
            }

            if (!tankPlayer.agent.hasPath && !tankPlayer.agent.pathPending || tankPlayer.agent.pathStatus == NavMeshPathStatus.PathInvalid)
            {
                tankPlayer.MoveTo(m_PowerUp.transform.position);
                return;
            }
        }

        private void ActiveAttackUpdate()
        {
            SelectTarget(true);

            if (!m_Target)
            {
                ChangeState(AIState.FindPowerup);
                return;
            }

            ObjectSpawner spawner = SelectPowerup();
            int numHitsLeft = NumOfHits(tankPlayer.health, tankPlayer.shield);
            int targetNumHitsLeft = NumOfHits(m_Target.health, m_Target.shield);

            if (spawner)
            {
                if (numHitsLeft != 0 && targetNumHitsLeft /  numHitsLeft >= 2)
                {
                    m_PowerUp = spawner;
                    tankPlayer.MoveTo(m_PowerUp.transform.position);
                    ChangeState(AIState.SimpleGoToPowerup);
                    return;
                }

                NavMesh.CalculatePath(tankPlayer.Position, spawner.transform.position, NavMesh.AllAreas, m_PathBuffer);
                float lengthToCollectible = PathLength(m_PathBuffer);

                NavMesh.CalculatePath(tankPlayer.Position, m_Target.Position, NavMesh.AllAreas, m_PathBuffer);
                float lengthToTarget = PathLength(m_PathBuffer);

                if ((lengthToTarget >= 10f && targetNumHitsLeft > 1) || lengthToTarget >= 15f)
                {
                    if (lengthToCollectible <= 10f || (spawner == m_SheildSpawner && lengthToCollectible <= 15f))
                    {
                        m_PowerUp = spawner;
                        tankPlayer.MoveTo(m_PowerUp.transform.position);
                        ChangeState(AIState.SimpleGoToPowerup);
                        return;
                    }
                }
            }

            bool isSightblocked = CheckObstaclesBetween(m_Target.Position, tankPlayer.Position, 0f);
            float linearDistToTarget = Vector3.Distance(m_Target.Position, tankPlayer.Position);

            //run away
            if (linearDistToTarget < m_EscapeDist && targetNumHitsLeft / numHitsLeft >= 2)
            {
                int steps = 10;

                //used if none of the points are traversible
                float maxDistFromEnemy = Mathf.Epsilon;
                Vector3 maxDistPoint = Vector3.zero;
                bool notBlocked = false;

                //used for traversible points
                float minDistToPlayer = Mathf.Infinity;
                Vector3 minDistDir = Vector3.zero;

                int i = steps;
                float angleStep = 360f / steps;

                NavMeshHit hitInfo;
                for (Vector3 dir = tankPlayer.Position - m_Target.Position; i >= 0; i--, dir = Quaternion.Euler(0f, angleStep, 0f) * dir)
                {
                    Vector3 idealpos = m_Target.Position + dir.normalized * m_EscapeDist;
                    if (NavMesh.Raycast(m_Target.Position, idealpos, out hitInfo, NavMesh.AllAreas))
                    {
                        Debug.DrawRay(m_Target.Position, dir, Color.red);
                        //Debug.Break();

                        if (maxDistFromEnemy < hitInfo.distance)
                        {
                            maxDistFromEnemy = hitInfo.distance;
                            maxDistPoint = hitInfo.position;
                        }
                    }
                    else
                    {
                        Debug.DrawRay(m_Target.Position, dir, Color.blue);
                        //Debug.Break();

                        notBlocked = true;

                        float dist = Vector3.Distance(tankPlayer.Position, idealpos);

                        if (minDistToPlayer > dist)
                        {
                            minDistToPlayer = dist;
                            minDistDir = dir.normalized;
                        }
                    }
                }

                if (!notBlocked)
                {
                    if (tankPlayer.agent.destination != maxDistPoint)
                    {
                        tankPlayer.agent.ResetPath();
                        //Vector3 dir = maxDistPoint - tankPlayer.Position;
                        //tankPlayer.SimpleMove(new Vector2(dir.x, dir.z));
                        tankPlayer.MoveTo(maxDistPoint);
                    }
                    else
                    {
                        tankPlayer.SimpleMove();
                    }
                }
                else
                {
                    Vector3 dest = m_Target.Position + minDistDir * m_EscapeDist;
                    if (tankPlayer.agent.destination != dest)
                    {
                        tankPlayer.agent.ResetPath();
                        //Vector3 dir = dest - tankPlayer.Position;
                        //tankPlayer.SimpleMove(new Vector2(dir.x, dir.z));
                        tankPlayer.MoveTo(dest);
                    }
                    else
                    {
                        tankPlayer.SimpleMove();
                    }
                }

                return;
            }

            //colliding or too close
            Collider c = m_Target.GetComponent<Collider>();
            if ((c && c.bounds.Intersects(m_Collider.bounds)) ||
                (!isSightblocked && linearDistToTarget < m_CombatDist))
            {
                int steps = 10;

                //used if none of the points are traversible
                float maxDistFromEnemy = Mathf.Epsilon;
                Vector3 maxDistPoint = Vector3.zero;
                bool notBlocked = false;

                //used for traversible points
                float minDistToPlayer = Mathf.Infinity;
                Vector3 minDistDir = Vector3.zero;

                int i = steps;
                float angleStep = 360f / steps;

                NavMeshHit hitInfo;
                for(Vector3 dir = tankPlayer.Position - m_Target.Position; i >= 0; i--, dir = Quaternion.Euler(0f, angleStep, 0f) * dir)
                {
                    Vector3 idealpos = m_Target.Position + dir.normalized * m_CombatDist;
                    if (NavMesh.Raycast(m_Target.Position, idealpos, out hitInfo, NavMesh.AllAreas))
                    {
                        Debug.DrawRay(m_Target.Position, dir, Color.red);
                        //Debug.Break();

                        if (maxDistFromEnemy < hitInfo.distance)
                        {
                            maxDistFromEnemy = hitInfo.distance;
                            maxDistPoint = hitInfo.position;
                        }
                    }
                    else
                    {
                        Debug.DrawRay(m_Target.Position, dir, Color.blue);
                        //Debug.Break();

                        notBlocked = true;

                        float dist = Vector3.Distance(tankPlayer.Position, idealpos);

                        if (minDistToPlayer > dist)
                        {
                            minDistToPlayer = dist;
                            minDistDir = dir.normalized;
                        }
                    }
                }

                if(!notBlocked)
                {
                    if(tankPlayer.agent.destination != maxDistPoint)
                    {
                        //Vector3 dir = maxDistPoint - tankPlayer.Position;
                        //tankPlayer.SimpleMove(new Vector2(dir.x, dir.z));
                        tankPlayer.MoveTo(maxDistPoint);
                    }
                    else
                    {
                        tankPlayer.SimpleMove();
                    }
                }
                else
                {
                    Vector3 dest = m_Target.Position + minDistDir * m_CombatDist;
                    if (tankPlayer.agent.destination != dest)
                    {
                        //Vector3 dir = dest - tankPlayer.Position;
                        //tankPlayer.SimpleMove(new Vector2(dir.x, dir.z));
                        tankPlayer.MoveTo(dest);
                    }
                    else
                    {
                        tankPlayer.SimpleMove();
                    }
                }

                return;
            }

            tankPlayer.MoveTo(m_Target.Position);
        }

        private void FindPowerupUpdate()
        {
            SelectTarget(false);

            if (m_Target)
            {
                ChangeState(AIState.ActivelyAttack);
                return;
            }

            if (!m_PowerUp || !m_PowerUp.obj || (!tankPlayer.agent.hasPath && !tankPlayer.agent.pathPending) || tankPlayer.agent.pathStatus == NavMeshPathStatus.PathInvalid)
            {
                m_PowerUp = SelectPowerup();

                if(!m_PowerUp)
                {
                    ChangeState(AIState.ActivelyAttack);
                    return;
                }

                tankPlayer.MoveTo(m_PowerUp.transform.position);
            }
        }

        //check the immediate proximity for objects of type T, return objects from the list that meet the constraint
        private T[] CheckSurroundingObjectsByType<T>(float radius, Predicate<T> constraint) where T : MonoBehaviour
        {
            RaycastHit[] hitInfos;

            Vector3 start = tankPlayer.Position + Vector3.up * 2f;
            hitInfos = Physics.SphereCastAll(start, radius, Vector3.down, start.y + 1f);

            List<T> l = new List<T>(hitInfos.Length);

            for (int i = 0; i < hitInfos.Length; i++)
            {
                T obj = hitInfos[i].collider.gameObject.GetComponent<T>();

                if (!obj) continue;
                if (obj.transform.root == gameObject.transform.root) continue;

                l.Add(obj);
            }

            T[] arr = l.ToArray();

            if (constraint != null)
                return Array.FindAll(arr, constraint);
            return arr;
        }

        #region find powerup update helpers
        private ObjectSpawner SelectPowerup()
        {
            if(m_SheildSpawner && tankPlayer.shield < m_ShieldAmount)
            {
                float timeBeforeShieldSpawn = m_NextSpawnTime - Time.time;
                NavMesh.CalculatePath(tankPlayer.Position, m_SheildSpawner.transform.position, NavMesh.AllAreas, m_PathBuffer);

                float timeToShieldSpawner = PathLength(m_PathBuffer) / m_TankSpeed;

                if (Mathf.Abs(timeBeforeShieldSpawn - timeToShieldSpawner) <= 2f)
                {
                    return m_SheildSpawner;
                }
            }

            Collectible[] collectibles = CheckSurroundingObjectsByType(m_ItemSensorRadius,
                (Collectible c) => c.gameObject.activeSelf);

            if (collectibles.Length > 0)
            {
                //SortByDist<Collectible> comparer = new SortByDist<Collectible>(tankPlayer);
                SortByAStarDist<Collectible> comparer = new SortByAStarDist<Collectible>(tankPlayer);

                //low health, prioritize finding health and sheild
                if(tankPlayer.health / (float) tankPlayer.maxHealth < 0.5)
                {
                    Collectible[] healthBoosts = Array.FindAll(collectibles, (Collectible c) => c.GetType() == typeof(PowerupHealth));
                    
                    if(healthBoosts.Length > 0)
                    {
                        Array.Sort(healthBoosts, comparer);
                    }

                    Collectible shield = Array.Find(collectibles, (Collectible c) => c.GetType() == typeof(PowerupShield));

                    if (shield)
                    {
                        return comparer.Compare(shield, healthBoosts[0]) < 0 ? shield.spawner : healthBoosts[0].spawner;
                    }
                    else
                    {
                        return healthBoosts[0].spawner;
                    }
                }

                Array.Sort(collectibles, comparer);

                List<Collectible> tempList = new List<Collectible>(collectibles.Length);
                int i;
                for(i=0; i<collectibles.Length; i++)
                {
                    if (collectibles[i].GetType() == typeof(PowerupShield))
                    {
                        PowerupShield shield = (PowerupShield)collectibles[i];
                        if (tankPlayer.shield == m_ShieldAmount)
                            continue;

                        if (!m_SheildSpawner)
                            m_SheildSpawner = shield.spawner;

                        //always prioritize sheild
                        return shield.spawner;
                    }

                    //prevent getting stuck trying to pick up a health item when on full health
                    if (collectibles[i].GetType() == typeof(PowerupHealth) && tankPlayer.health == tankPlayer.maxHealth)
                    {
                        continue;
                    }

                    //prevent getting stuck trying to pick up an item the tank already has
                    if(collectibles[i].GetType() == typeof(PowerupBullet)) //TODO: hardcoded 1
                    {
                        PowerupBullet powerupBullet = (PowerupBullet)collectibles[i];

                        if(tankPlayer.ammo == powerupBullet.amount && tankPlayer.currentBullet == powerupBullet.bulletIndex)
                            continue;
                    }

                    tempList.Add(collectibles[i]);
                }

                return tempList.Count > 0 ? tempList[0].spawner : null;
            }
            else
            {
                return null;
            }
        }
        #endregion

        #region attack update helpers
        private void PreciseShootTarget()
        {
            if(Vector3.Distance(m_Target.Position, tankPlayer.Position) < 5f)
            {
                tankPlayer.AimAndShoot(m_Target.Position);
                return;
            }

            float cosTheta = Vector3.Dot((tankPlayer.Position - m_Target.Position).normalized, m_Target.Velocity.normalized);

            if(Mathf.Approximately(cosTheta, 1f) || Mathf.Approximately(cosTheta, -1f))
            {
                tankPlayer.AimAndShoot(m_Target.Position);
                return;
            }

            Vector3 shotPosToEnemy = m_Target.Position - tankPlayer.Position;
            float dist = shotPosToEnemy.magnitude;
            float sqrDist = shotPosToEnemy.sqrMagnitude;

            float a = (m_BulletSpeed * m_BulletSpeed) - (m_TankSpeed * m_TankSpeed);
            float b = 2 * dist * m_TankSpeed * cosTheta;
            float c = -(dist * dist);

            float t1, t2, hittime = float.NaN;
            SolveQuadEquation(a, b, c, out t1, out t2);

            if ((t1 == float.NaN && t2 == float.NaN) || (t1 < 0 && t2 < 0)) return;
            else if (t1 < 0 || t1 == float.NaN) hittime = t2;
            else if (t2 < 0 || t2 == float.NaN) hittime = t1;
            else hittime = Mathf.Min(t1, t2);

            //Vector3 desiredDir = m_Target.Velocity + (shotPosToEnemy / hittime);
            //Debug.Log(desiredDir.magnitude.ToString() + " , " + m_BulletSpeed.ToString());

            Vector3 hitPos = hittime * m_TankSpeed * m_Target.Velocity.normalized + m_Target.Position;

            if (!CheckObstaclesBetween(tankPlayer.Position, hitPos, 0f))
            {
                Debug.DrawLine(tankPlayer.Position, hitPos, Color.green);
                tankPlayer.AimAndShoot(hitPos);
            }
            else
            {
                Debug.DrawLine(tankPlayer.Position, hitPos, Color.red);
            }
        }
        /*
        private void PreciseShootTarget()
        {
            MotionInfo targetMotion = new MotionInfo(MotionType.NavMeshAgent, m_Target.Position, m_Target.Velocity, m_Target.gameObject);

            Vector3 bulletDir = targetMotion.bounds.center - tankPlayer.shotPos.position;
            MotionInfo myBulletMotion = new MotionInfo(MotionType.Linear, tankPlayer.shotPos.position, bulletDir.normalized * m_BulletSpeed, Vector3.one * (m_BulletRadius * 2f));

            //initial guess
            float step;
            Bounds targetBounds = targetMotion.bounds;
            Bounds bulletBounds;

            int tolerance = 0;
            bool success = false;
            while (tolerance < 50)
            {
                step = Vector3.Distance(targetBounds.center, tankPlayer.shotPos.position) / (m_BulletSpeed * Time.fixedDeltaTime);
                targetBounds = targetMotion.PredictPosition(Mathf.CeilToInt(step));

                myBulletMotion.velocity = (targetBounds.center - tankPlayer.shotPos.position).normalized * m_BulletSpeed;
                bulletBounds = myBulletMotion.PredictPosition(Mathf.CeilToInt(step));
                tolerance++;

                if (targetBounds.Intersects(bulletBounds))
                {
                    success = true;
                    break;
                }
            }

            if(!success)
            {
                //Debug.DrawLine(tankPlayer.shotPos.position, targetBounds.center, Color.red);
            }
            else
            {
                Debug.DrawLine(tankPlayer.shotPos.position, targetBounds.center, Color.green);
                tankPlayer.AimAndShoot(targetBounds.center);
            }
        }
        */

        //target selection:
        //degree of threat:
        //1. about to collide
        //2. closest
        //3. low health
        //always choose the enemy with the least amount of health + sheild
        private void SelectTarget(bool seek)
        {
            if(seek)
            {
                enemys = CheckSurroundingObjectsByType(m_PlayerSensorRadius,
                    (BasePlayer player) => player.teamIndex != tankPlayer.teamIndex && player.IsAlive);
            }
            else
            {
                enemys = CheckSurroundingObjectsByType(m_PlayerSensorRadius,
                    (BasePlayer player) => player.teamIndex != tankPlayer.teamIndex && player.IsAlive && !CheckObstaclesBetween(player.Position, tankPlayer.Position, 0f));
            }

            if (enemys.Length > 0)
            {
                int minHealth = NumOfHits(enemys[0].health, enemys[0].shield);
                int minHealthIndex = 0;

                int minColTime = int.MaxValue;
                int minColTimeIndex = -1;

                float minDist = float.MaxValue;
                int minDistIndex = -1;

                for (int i = 0; i < enemys.Length; i++)
                {
                    int health = NumOfHits(enemys[i].health, enemys[i].shield);
                    if (minHealth > health)
                    {
                        minHealth = health;
                        minHealthIndex = i;
                    }

                    MotionInfo enemyMotion = new MotionInfo(MotionType.NavMeshAgent, enemys[i].Position, enemys[i].Velocity, enemys[i].gameObject);
                    int colTime = WillCollide(enemyMotion, m_PlayerMotionInfo, m_PredictionStep);

                    if (colTime != -1)
                    {
                        if (minColTime > colTime)
                        {
                            minColTime = colTime;
                            minColTimeIndex = i;
                        }
                    }

                    float dist = Vector3.Distance(tankPlayer.Position, enemys[i].Position);
                    if(dist < minDist)
                    {
                        minDist = dist;
                        minDistIndex = i;
                    }
                }

                //someone is about to run into us
                if (minColTimeIndex != -1)
                {
                    m_Target = enemys[minColTimeIndex];
                }
                else
                {
                    if(minDistIndex != -1 && minDist < m_CombatDist + 5f)
                    {
                        m_Target = enemys[minDistIndex];
                    }
                    else
                    {
                        if (seek)
                        {
                            m_Target = enemys[minHealthIndex];
                        }
                        else
                        {
                            m_Target = null;
                        }
                    }
                }
            }
            else
            {
                m_Target = null;
            }
        }
        #endregion

        #region dodge update helpers
        private void Dodge()
        {
            if (m_ThreatLastFrame == m_Threat)
            {
                tankPlayer.SimpleMove();
                return;
            }

            Vector3 threatToPlayer = tankPlayer.Position - m_Threat.Position;
            Vector3 playerRight = tankPlayer.transform.TransformDirection(Vector3.right);
            bool behind = Vector3.Cross(playerRight, threatToPlayer).y < 0;
            float angle = Vector3.SignedAngle(threatToPlayer, m_Threat.Velocity, Vector3.up);
            Vector3 dir;
            Vector2 lookRotation;

            if (angle > 0f || Mathf.Approximately(angle, 0.0f))
            {
                if (behind)
                {
                    //Debug.Log("behind and from left");
                    dir = Vector3.Cross(m_Threat.Velocity, Vector3.up);
                }
                else
                {
                   // Debug.Log("front and from right");
                    dir = Vector3.Cross(m_Threat.Velocity, Vector3.down);
                }
            }
            else
            {
                if (behind)
                {
                    //Debug.Log("behind and from right");
                    dir = Vector3.Cross(m_Threat.Velocity, Vector3.down);
                }
                else
                {
                    //Debug.Log("front and from left");
                    dir = Vector3.Cross(m_Threat.Velocity, Vector3.up);
                }
            }

            dir = Vector3.ProjectOnPlane(dir, Vector3.up);
            dir.Normalize();

            //rotate the dodge direction to find a direction with max dodge space
            float[] maxDist = { Mathf.Epsilon, Mathf.Epsilon }; //max dist for dir and -dir
            float[] maxDistAngleOffset = { -1, -1 }; //corresponding max distance
            bool notBlocked = false;

            Vector3 obstaclePos;
            Vector3 temp;

            Vector3[] directions = { dir, -dir };
            int[] step = { 5, -5, 5, -5 };
            int i = 0, j;

            float minDodgeDistance = m_Collider.size.z * 1.5f;
            Vector3 verticalOffset = Vector3.up * (m_Collider.size.x * 0.55f);
            Vector3 offsetedPos = tankPlayer.Position + verticalOffset;

            for (j=0; j < 4; j++)
            {
                i = 0;
                for (temp = directions[j/2]; i < Mathf.Abs(45 / step[j]); i++, temp = Quaternion.Euler(0, step[j], 0) * temp)
                {
                    if (CheckObstaclesBetween(offsetedPos, offsetedPos + temp * minDodgeDistance, out obstaclePos, m_Collider.size.x / 2f))
                    {
                        Debug.DrawLine(offsetedPos, obstaclePos, j/2 == 1 ? Color.yellow : Color.red);

                        float dist = XZDistance(obstaclePos, tankPlayer.Position);

                        if (maxDist[j/2] < dist)
                        {
                            maxDist[j/2] = dist;
                            maxDistAngleOffset[j/2] = step[j] * i;
                        }
                    }
                    else
                    {
                        Debug.DrawLine(offsetedPos, offsetedPos + temp * minDodgeDistance, Color.blue);

                        notBlocked = true;
                        dir = Quaternion.Euler(0, step[j] * i, 0) * directions[j/2];

                        break;
                    }
                }

                if (notBlocked) break;
            }

            if (!notBlocked)
            {
                j = maxDist[0] > maxDist[1] ? 0 : 1;
                dir = Quaternion.Euler(0, step[j] * maxDist[j], 0) * dir;
            }

            //Debug.Break();
            lookRotation = new Vector2(dir.x, dir.z);
            tankPlayer.SimpleMove(lookRotation);
            //tankPlayer.MoveTo(tankPlayer.Position + tankPlayer.transform.forward * m_DodgeExtent);
        }
        #endregion

        #region utility
        private class SortByAStarDist<T> : IComparer<T> where T : MonoBehaviour
        {
            private BasePlayer tankPlayer;
            private NavMeshPath pathA = new NavMeshPath(), pathB = new NavMeshPath();

            private SortByAStarDist() { }

            public SortByAStarDist(BasePlayer tankPlayer)
            {
                this.tankPlayer = tankPlayer;
            }

            public int Compare(T x, T y)
            {
                NavMesh.CalculatePath(tankPlayer.Position, x.transform.position, NavMesh.AllAreas, pathA);
                NavMesh.CalculatePath(tankPlayer.Position, y.transform.position, NavMesh.AllAreas, pathB);

                float pathLengthA = PathLength(pathA), pathLengthB = PathLength(pathB);
                if (Mathf.Approximately(pathLengthA, pathLengthB)) return 0;
                else if (pathLengthA < pathLengthB) return -1;
                else return 1;
            }
        }


        private class SortByDist<T> : IComparer<T> where T : MonoBehaviour
        {
            private BasePlayer tankPlayer;

            private SortByDist() { }

            public SortByDist(BasePlayer tankPlayer)
            {
                this.tankPlayer = tankPlayer;
            }

            public int Compare(T x, T y)
            {
                float dist_x = Vector3.Distance(tankPlayer.Position, x.transform.position);
                float dist_y = Vector3.Distance(tankPlayer.Position, x.transform.position);

                if (Mathf.Approximately(dist_x, dist_y)) return 0;
                else if (dist_x < dist_y) return -1;
                else return 1;
            }
        }

        private static void SolveQuadEquation(float a, float b, float c, out float sol1, out float sol2)
        {
            float insideSqrt = (b * b) - 4 * a * c;
            if(insideSqrt < 0)
            {
                sol1 = float.NaN;
                sol2 = float.NaN;
            }
            else
            {
                float sqrt = Mathf.Sqrt(insideSqrt);
                sol1 = (-b + sqrt) / (2 * a);
                sol2 = (-b - sqrt) / (2 * a);
            }
        }

        private static int NumOfHits(int health, int shield)
        {
            return shield + health / 3 + 1;
        }

        private static float PathLength(NavMeshPath path)
        {
            float ret = 0f;
            for (int i = 0; i < path.corners.Length - 1; i++)
            {
                ret += Vector3.Distance(path.corners[i], path.corners[i + 1]);
                Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.black);
            }

            return ret;
        }

        private static bool CheckObstaclesBetween(Vector3 pos1, Vector3 pos2, out Vector3 obstaclePosition, Vector3 dimension)
        {
            obstaclePosition = default(Vector3);
            RaycastHit hitInfo;

            if (dimension == Vector3.zero)
            {
                //RaycastHit hitInfo;
                if (Physics.Linecast(pos1, pos2, out hitInfo, 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    obstaclePosition = hitInfo.point;
                    return true;
                }

                return false;
            }
            else
            {
                if (Physics.BoxCast(pos1, dimension * 0.5f, pos2 - pos1, out hitInfo, Quaternion.identity, Vector3.Distance(pos1, pos2), 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    obstaclePosition = hitInfo.point;
                    return true;
                }

                return false;
            }
        }

        private static bool CheckObstaclesBetween(Vector3 pos1, Vector3 pos2, float radius)
        {
            if(Mathf.Approximately(radius, 0f))
            {
                //RaycastHit hitInfo;
                if (Physics.Linecast(pos1, pos2, 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    return true;
                }

                return false;
            }
            else
            {
                Ray ray = new Ray(pos1, pos2 - pos1);
                if(Physics.SphereCast(ray, radius, Vector3.Distance(pos1, pos2), 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    return true;
                }

                return false;
            }
        }

        //given two positions, check if there is an obstacle between the two
        //if there is an obstacle, obstacle position is filled
        private static bool CheckObstaclesBetween(Vector3 pos1, Vector3 pos2, out Vector3 obstaclePosition, float radius)
        {
            obstaclePosition = default(Vector3);
            RaycastHit hitInfo;

            if (Mathf.Approximately(radius, 0f))
            {
                if (Physics.Linecast(pos1, pos2, out hitInfo, 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    obstaclePosition = hitInfo.point;
                    return true;
                }

                return false;
            }
            else
            {
                if (Physics.SphereCast(pos1, radius, pos2 - pos1, out hitInfo, Vector3.Distance(pos1, pos2), 1 << 0, QueryTriggerInteraction.Ignore))
                {
                    obstaclePosition = hitInfo.point;
                    return true;
                }

                return false;
            }
        }

        //given two objects' motion infos and their sizes, predict if they will collide
        //returns the frame number in which the collision occurs
        //returns -1 if the collision will not happen within time step
        private static int WillCollide(MotionInfo motionInfo1, MotionInfo motionInfo2, int timeStep)
        {
            //Debug.DrawLine(motionInfo1.bounds.center, motionInfo1.PredictPosition(timeStep).center, Color.blue);
            //Debug.DrawLine(motionInfo2.bounds.center, motionInfo2.PredictPosition(timeStep).center, Color.red);

            //simulate
            for (int i=1; i<=timeStep; i++)
            {
                if (motionInfo1.PredictPosition(i).Intersects(motionInfo2.PredictPosition(i)))
                {
                    return i;
                }
            }

            //Debug.Log("will not collide");

            return -1;
        }

        private static float XZDistance(Vector3 x, Vector3 y)
        {
            return Vector3.Distance(Vector3.ProjectOnPlane(x, Vector3.up), Vector3.ProjectOnPlane(y, Vector3.up));
        }

        #endregion

        #region debug
        private void OnGUI()
        {
            GUILayout.Box("num of surrounding bullets: " +
                (bullets == null || bullets.Length == 0 ? "none" : bullets.Length.ToString()));
            GUILayout.Box("current threat: " +
                (m_Threat ? m_Threat.name : "none"));
            GUILayout.Box("current state: " + m_CurState.ToString());
            GUILayout.Box("sheild spawn countdown: " + (m_NextSpawnTime - Time.time).ToString());
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
            Gizmos.DrawWireSphere(tankPlayer.Position, m_BulletSensorRadius);
            Gizmos.DrawWireSphere(tankPlayer.Position, m_PlayerSensorRadius);

            if (m_Target)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(m_Target.Position, 3.0f);
            }

            if (m_Threat)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(m_Threat.Position, 0.5f);
            }

            if (m_PowerUp)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(m_PowerUp.transform.position, 2.0f);
            }

            Gizmos.color = Color.black;
            Gizmos.DrawIcon(tankPlayer.agent.destination + Vector3.up * 1.5f, "destination");
            Gizmos.DrawCube(tankPlayer.agent.destination, new Vector3(1, 1, 1));

            /*
            Gizmos.color = Color.green;
            Gizmos.DrawCube(m_PlayerMotionInfo.PredictPosition(50).center, new Vector3(2, 2, 2));

            Gizmos.color = Color.cyan;
            Vector3[] corners = tankPlayer.agent.path.corners;
            foreach (Vector3 waypoint in corners)
            {
                Gizmos.DrawCube(waypoint, new Vector3(2, 2, 2));
            }

            Gizmos.color = Color.red;
            Gizmos.DrawCube(tankPlayer.agent.steeringTarget, new Vector3(2, 2, 2));
            */
        }
        #endregion
    }
}
